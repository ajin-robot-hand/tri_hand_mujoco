from pathlib import Path
import time
import mujoco
import mujoco.viewer

XML_PATH = Path(__file__).parent / "hello.xml"
model = mujoco.MjModel.from_xml_path(str(XML_PATH))
data = mujoco.MjData(model)

# 처음 시작할 때도 수평 속도 1.0 m/s로 던지기
data.qvel[0] = 1.0

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # 물리 연산 1스텝 진행
        mujoco.mj_step(model, data)

        # 바닥에 닿아 굴러가서 저 멀리(X > 2m) 벗어나면 처음 위치로 리셋!
        if data.qpos[0] > 2.0:
            mujoco.mj_resetData(model, data)  # 속도 및 상태 깨끗이 초기화
            data.qpos[2] = 1.0                # 다시 높이 1m로
            data.qvel[0] = 1.0                # 옆으로 던지는 속도 1.0 m/s
            mujoco.mj_forward(model, data)    # 좌표계 즉시 갱신

        # 뷰어 화면 갱신
        viewer.sync()

        # 실시간 속도 동기화
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)