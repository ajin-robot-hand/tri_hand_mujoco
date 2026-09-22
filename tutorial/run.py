"""MuJoCo 튜토리얼 범용 실행 스크립트.

사용법:
  python run.py                          # 기본 예제(01_hello.xml) 뷰어 실행
  python run.py 02_joints_tendon.xml     # 특정 XML 파일 뷰어 실행
  python run.py 02                       # 번호/키워드만으로도 자동 매칭
  python run.py 01 --headless            # GUI 없이 터미널에서 물리 연산 검증
"""
import argparse
import sys
import time
from pathlib import Path

import mujoco

TUTORIAL_DIR = Path(__file__).resolve().parent


def resolve_xml_path(target: str) -> Path:
    """입력받은 문자열로부터 XML 파일 경로를 유연하게 탐색합니다."""
    # 1. 전달된 경로 그대로 확인
    path = Path(target)
    if path.is_file():
        return path

    # 2. tutorial 디렉토리 내부에서 확인
    candidate = TUTORIAL_DIR / target
    if candidate.is_file():
        return candidate

    # 3. .xml 확장자 누락 시 보완
    if not target.endswith(".xml"):
        candidate_xml = TUTORIAL_DIR / f"{target}.xml"
        if candidate_xml.is_file():
            return candidate_xml

        # 4. '01', '02' 등 접두사/키워드 검색
        matches = list(TUTORIAL_DIR.glob(f"*{target}*.xml"))
        if matches:
            return sorted(matches)[0]

    # 사용 가능한 파일 목록 수집 후 에러 발생
    available = [f.name for f in sorted(TUTORIAL_DIR.glob("*.xml"))]
    print(f"[Error] '{target}'에 해당하는 XML 파일을 찾을 수 없습니다.")
    print(f"사용 가능한 튜토리얼 XML 목록: {', '.join(available)}")
    sys.exit(1)


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData):
    import mujoco.viewer

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()

            dt = model.opt.timestep - (time.time() - step_start)
            if dt > 0:
                time.sleep(dt)


def run_headless(model: mujoco.MjModel, data: mujoco.MjData, seconds: float = 3.0):
    steps = int(seconds / model.opt.timestep)
    print(f"== Headless 모드 실행 ({seconds}초, {steps} 스텝) ==")
    for i in range(steps):
        mujoco.mj_step(model, data)
        if i % int(0.5 / model.opt.timestep) == 0:
            print(f"t={data.time:4.2f}s | ncon(충돌수)={data.ncon:2d} | "
                  f"qpos={data.qpos[:3].round(2)}")
    print("== 시뮬레이션 정상 완료 ==")


def main():
    parser = argparse.ArgumentParser(description="MuJoCo Tutorial Runner")
    parser.add_argument(
        "xml",
        nargs="?",
        default="01_hello.xml",
        help="실행할 XML 파일명 (기본값: 01_hello.xml)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="뷰어 없이 터미널에서 물리 연산만 실행",
    )
    args = parser.parse_args()

    xml_path = resolve_xml_path(args.xml)
    print(f"로드 중: {xml_path.name} ({xml_path})")

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    if args.headless:
        run_headless(model, data)
    else:
        run_viewer(model, data)


if __name__ == "__main__":
    main()