"""tri_hand MuJoCo HTTP 제어 서버: 외부에서 받은 목표 각도로 관절을 위치 제어.

사용법
  python server.py              # 뷰어 + 서버 (macOS는 mjpython server.py)
  python server.py --headless   # 뷰어 없이 서버만 (WSL2/서버 등)
  python server.py --headless --host 0.0.0.0   # 같은 LAN의 다른 기기에서도 접속 허용

API (기본 http://127.0.0.1:8000)
  GET  /         대시보드 (웹 브라우저)
  GET  /state    현재 시뮬레이션 시각, 관절 각도/속도/힘, 목표 각도, 게인(kp/kv/토크 한계), 접촉 수
  GET  /joints   관절별 ctrlrange, forcerange, close_sign
  GET  /limits   프리셋 kp/kv/torque_limit 허용 범위
  POST /joints   목표 각도 지정 (rad). 일부 관절만 보내도 됨
                 예) curl -X POST localhost:8000/joints \
                       -H 'Content-Type: application/json' -d '{"A_j1": -0.3, "B_j1": 0.3}'
  POST /grasp    {"amount": 0~1} 로 전 손가락 오므림 정도 지정 (A_j0 제외)
  POST /reset    시뮬레이션 상태 초기화 (게인은 유지)
  GET  /presets              저장된 프리셋 목록
  PUT  /presets/{name}       프리셋 생성/수정 (7개 관절 모두 target, kp, kv, torque_limit). 적용하지 않음
  POST /presets/{name}/apply 저장된 프리셋을 시뮬레이션에 적용
"""
import argparse
import json
import math
import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi import Path as PathParam
from fastapi.exceptions import RequestValidationError
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel, ConfigDict, Field, RootModel, ValidationError, model_validator

from run_sim import CLOSE_SIGN, Q_CLOSE

SCENE = Path(__file__).with_name("scene.xml")
DASHBOARD = Path(__file__).with_name("dashboard.html")
PRESETS_PATH = Path(__file__).with_name("presets.json")   # --presets 로 변경 가능

model = mujoco.MjModel.from_xml_path(str(SCENE))
data = mujoco.MjData(model)
lock = threading.Lock()   # 시뮬레이션 스레드와 HTTP 요청 스레드가 model/data를 함께 쓰므로 보호
presets_lock = threading.Lock()   # 프리셋 파일 읽기-수정-쓰기 보호

# 프리셋 허용 범위. kp/kv 상한은 이 범위 전체에서 ±ctrlrange 스텝 입력 시 발산 없음을 확인한 값
KP_RANGE = (0.0, 100.0)    # kp > 0
KV_RANGE = (0.0, 5.0)
TORQUE_LIMIT_MAX = float(model.actuator_forcerange[:, 1].min())   # XL430 정격 1.4 N·m (hand.xml)


@dataclass(frozen=True)
class Joint:
    act: int                       # 액추에이터 id
    qpos: int                      # qpos 주소
    dof: int                       # qvel/dof 주소
    ctrlrange: tuple[float, float]
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
            close_sign=CLOSE_SIGN[finger] if suffix in Q_CLOSE else None,
        )
    return joints


JOINTS = _build_joints()

app = FastAPI(title="tri_hand control")


@app.exception_handler(RequestValidationError)
def _validation_error(request, exc):
    # 기본 응답은 입력값(input)을 되돌려 주는데, NaN이 들어오면 JSON 직렬화가 실패해 500이 됨
    return JSONResponse(status_code=422, content={"detail": [
        {k: v for k, v in e.items() if k in ("loc", "msg", "type")} for e in exc.errors()]})


def _gains(j: Joint) -> dict:
    """position 액추에이터: force = kp*(ctrl - q) - kv*qdot → gainprm[0]=kp, biasprm[1]=-kp, biasprm[2]=-kv"""
    return {
        "kp": float(model.actuator_gainprm[j.act, 0]),
        "kv": float(-model.actuator_biasprm[j.act, 2]),
        "torque_limit": float(model.actuator_forcerange[j.act, 1]),
    }


def _state() -> dict:
    return {
        "time": data.time,
        "joints": {n: float(data.qpos[j.qpos]) for n, j in JOINTS.items()},
        "targets": {n: float(data.ctrl[j.act]) for n, j in JOINTS.items()},
        "velocities": {n: float(data.qvel[j.dof]) for n, j in JOINTS.items()},
        "forces": {n: float(data.actuator_force[j.act]) for n, j in JOINTS.items()},
        "gains": {n: _gains(j) for n, j in JOINTS.items()},
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
        n: {"ctrlrange": list(j.ctrlrange), "forcerange": model.actuator_forcerange[j.act].tolist(),
            "close_sign": j.close_sign}
        for n, j in JOINTS.items()
    }


@app.get("/limits")
def limits():
    # kp, torque_limit 하한은 제외(> 0), kv 하한은 포함(>= 0)
    return {"kp": list(KP_RANGE), "kv": list(KV_RANGE), "torque_limit": [0.0, TORQUE_LIMIT_MAX],
            "exclusive_min": ["kp", "torque_limit"]}


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


class Motor(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    target: float                                            # rad, 관절별 ctrlrange는 Preset에서 검사
    kp: float = Field(gt=KP_RANGE[0], le=KP_RANGE[1])        # N·m/rad
    kv: float = Field(ge=KV_RANGE[0], le=KV_RANGE[1])        # N·m·s/rad
    torque_limit: float = Field(gt=0, le=TORQUE_LIMIT_MAX)   # N·m, forcerange = ±torque_limit


class Preset(RootModel[dict[str, Motor]]):
    """7개 관절 전체 값. 일부만 있으면 적용 후 상태가 이전 값에 따라 달라지므로 거부."""

    @model_validator(mode="after")
    def _check(self):
        names = set(self.root)
        if names != set(JOINTS):
            raise ValueError(f"관절 7개가 모두 필요: 누락 {sorted(set(JOINTS) - names)}, "
                             f"알 수 없음 {sorted(names - set(JOINTS))}")
        for n, m in self.root.items():
            lo, hi = JOINTS[n].ctrlrange
            if not lo <= m.target <= hi:
                raise ValueError(f"{n} target {m.target}가 범위 [{lo}, {hi}] 밖")
        return self


PresetName = PathParam(min_length=1, max_length=40, pattern=r"^[\w-]+(?: [\w-]+)*$")


def _load_presets() -> dict[str, Preset]:
    # exists()로 먼저 확인하지 않음: 접근 거부도 False로 보여 빈 목록으로 오인하기 때문
    try:
        raw = json.loads(PRESETS_PATH.read_text(encoding="utf-8"))
        return {name: Preset.model_validate(p) for name, p in raw.items()}
    except FileNotFoundError:
        return {}   # 처음 실행: 아직 저장된 프리셋 없음
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, AttributeError, ValidationError) as e:
        # 손으로 고치다 깨진 파일은 덮어쓰지 않고, 사람이 고치도록 프리셋 요청만 실패시킴
        raise HTTPException(500, f"프리셋 파일 {PRESETS_PATH.name}을 읽을 수 없음 ({type(e).__name__}). "
                                 f"파일을 고치거나 지운 뒤 다시 시도하세요")


def _save_presets(presets: dict[str, Preset]):
    # 임시 파일에 쓰고 교체해서, 쓰는 도중 서버가 죽어도 기존 파일이 깨지지 않게 함
    tmp = PRESETS_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps({n: p.model_dump() for n, p in presets.items()}, ensure_ascii=False, indent=2),
                   encoding="utf-8")
    os.replace(tmp, PRESETS_PATH)


@app.get("/presets")
def list_presets():
    with presets_lock:
        return {n: p.model_dump() for n, p in _load_presets().items()}


@app.put("/presets/{name}")
def save_preset(preset: Preset, name: str = PresetName):
    """저장만 함. 시뮬레이션에는 /presets/{name}/apply 를 호출해야 반영됨."""
    with presets_lock:
        presets = _load_presets()
        created = name not in presets
        presets[name] = preset
        _save_presets(presets)
    return {"name": name, "created": created}


@app.post("/presets/{name}/apply")
def apply_preset(name: str = PresetName):
    with presets_lock:
        preset = _load_presets().get(name)
    if preset is None:
        raise HTTPException(404, f"프리셋 없음: {name}")
    with lock:
        for n, m in preset.root.items():
            i = JOINTS[n].act
            model.actuator_gainprm[i, 0] = m.kp
            model.actuator_biasprm[i, 1] = -m.kp
            model.actuator_biasprm[i, 2] = -m.kv
            model.actuator_forcerange[i] = (-m.torque_limit, m.torque_limit)
            data.ctrl[i] = m.target
        return _state()


@app.get("/")
def dashboard():
    return FileResponse(DASHBOARD)


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
    ap.add_argument("--presets", type=Path, default=PRESETS_PATH, help="프리셋 JSON 파일 경로")
    args = ap.parse_args()
    PRESETS_PATH = args.presets
    # 뷰어는 메인 스레드에서 돌아야 하므로(macOS) HTTP 서버를 별도 스레드로 띄움
    server = threading.Thread(
        target=uvicorn.run, args=(app,), kwargs={"host": args.host, "port": args.port}, daemon=True)
    server.start()
    try:
        (run_headless if args.headless else run_viewer)()
    except KeyboardInterrupt:
        pass
