"""tri_hand MuJoCo HTTP 제어 서버: 외부에서 받은 목표 각도로 관절을 위치 제어.

사용법
  python server.py              # 뷰어 + 서버 (macOS는 mjpython server.py)
  python server.py --headless   # 뷰어 없이 서버만 (WSL2/서버 등)

API (기본 http://127.0.0.1:8000)
  GET  /state    현재 시뮬레이션 시각, 관절 각도/속도/힘, 목표 각도, 접촉 수
  GET  /joints   관절별 ctrlrange, forcerange, close_sign
  POST /joints   목표 각도 지정 (rad). 일부 관절만 보내도 됨
                 예) curl -X POST localhost:8000/joints \
                       -H 'Content-Type: application/json' -d '{"A_j1": -0.3, "B_j1": 0.3}'
  POST /grasp    {"amount": 0~1} 로 전 손가락 오므림 정도 지정 (A_j0 제외)
  POST /reset    시뮬레이션 상태 초기화
"""
import argparse
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel, Field

from run_sim import CLOSE_SIGN, Q_CLOSE

SCENE = Path(__file__).with_name("scene.xml")

model = mujoco.MjModel.from_xml_path(str(SCENE))
data = mujoco.MjData(model)
lock = threading.Lock()   # 시뮬레이션 스레드와 HTTP 요청 스레드가 data를 함께 쓰므로 보호


@dataclass(frozen=True)
class Joint:
    act: int                       # 액추에이터 id
    qpos: int                      # qpos 주소
    dof: int                       # qvel/dof 주소
    ctrlrange: tuple[float, float]
    forcerange: tuple[float, float]
    close_sign: float | None       # 오므림 방향 부호. 요(A_j0)는 대상 아님


def _build_joints() -> dict[str, Joint]:
    joints = {}
    for i in range(model.nu):
        name = model.actuator(i).name.removesuffix("_act")   # 액추에이터 이름은 "<관절>_act"
        finger, _, suffix = name.partition("_")
        j = model.joint(name)
        joints[name] = Joint(
            act=i,
            qpos=j.qposadr[0],
            dof=j.dofadr[0],
            ctrlrange=tuple(model.actuator_ctrlrange[i]),
            forcerange=tuple(model.actuator_forcerange[i]),
            close_sign=CLOSE_SIGN[finger] if suffix in Q_CLOSE else None,
        )
    return joints


JOINTS = _build_joints()

app = FastAPI(title="tri_hand control")


def _state() -> dict:
    return {
        "time": data.time,
        "joints": {n: float(data.qpos[j.qpos]) for n, j in JOINTS.items()},
        "targets": {n: float(data.ctrl[j.act]) for n, j in JOINTS.items()},
        "velocities": {n: float(data.qvel[j.dof]) for n, j in JOINTS.items()},
        "forces": {n: float(data.actuator_force[j.act]) for n, j in JOINTS.items()},
        "contacts": data.ncon,
    }


def _clip_apply(targets: dict[str, float]) -> dict:
    """ctrlrange로 잘라서 적용. /joints, /grasp가 공유."""
    applied, clipped = {}, []
    with lock:
        for name, q in targets.items():
            j = JOINTS[name]
            lo, hi = j.ctrlrange
            c = float(np.clip(q, lo, hi))
            if c != q:
                clipped.append(name)
            data.ctrl[j.act] = applied[name] = c
    return {"applied": applied, "clipped": clipped}


@app.get("/state")
def get_state():
    with lock:
        return _state()


@app.get("/joints")
def list_joints():
    return {
        n: {"ctrlrange": list(j.ctrlrange), "forcerange": list(j.forcerange), "close_sign": j.close_sign}
        for n, j in JOINTS.items()
    }


@app.post("/joints")
def set_joints(targets: dict[str, float]):
    unknown = set(targets) - set(JOINTS)
    if unknown:
        raise HTTPException(400, f"알 수 없는 관절: {sorted(unknown)}. 사용 가능: {list(JOINTS)}")
    non_finite = sorted(n for n, q in targets.items() if not math.isfinite(q))
    if non_finite:
        raise HTTPException(400, f"유한하지 않은 값: {non_finite}")
    return _clip_apply(targets)


class GraspRequest(BaseModel):
    amount: float = Field(ge=0, le=1)   # NaN도 ge/le 비교에서 걸러짐


@app.post("/grasp")
def grasp(req: GraspRequest):
    targets = {
        n: j.close_sign * Q_CLOSE[n.rpartition("_")[2]] * req.amount
        for n, j in JOINTS.items() if j.close_sign is not None
    }
    return _clip_apply(targets)


@app.post("/reset")
def reset():
    with lock:
        mujoco.mj_resetData(model, data)   # ctrl도 함께 0으로 초기화됨
        mujoco.mj_forward(model, data)
        return _state()


def step_realtime(last: float) -> float:
    """직전 tick 이후 경과한 벽시계 시간만큼만 진행 (한 번에 0.05s로 제한:
    reset 등으로 data.time이 벽시계보다 많이 뒤처져도 한꺼번에 몰아서 스텝하지 않음).
    step은 timestep 단위로만 진행되므로 target을 넘겨 딛는 만큼 last를 함께 당겨서,
    다음 tick의 목표 시간을 깎아 평균 속도가 실시간에 맞게 자기보정되도록 함."""
    now = time.time()
    with lock:
        t_start = data.time
        target = t_start + min(now - last, 0.05)
        while data.time < target:
            mujoco.mj_step(model, data)
        last += data.time - t_start
    return last


def run_headless():
    last = time.time()
    while True:
        last = step_realtime(last)
        time.sleep(model.opt.timestep)


def run_viewer():
    import mujoco.viewer
    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
        v.cam.fixedcamid = model.camera("iso").id
        last = time.time()
        while v.is_running():
            last = step_realtime(last)
            with lock:
                v.sync()
            time.sleep(1 / 60)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()
    # 뷰어는 메인 스레드에서 돌아야 하므로(macOS) HTTP 서버를 별도 스레드로 띄움
    server = threading.Thread(
        target=uvicorn.run, args=(app,), kwargs={"host": args.host, "port": args.port}, daemon=True)
    server.start()
    try:
        (run_headless if args.headless else run_viewer)()
    except KeyboardInterrupt:
        pass
