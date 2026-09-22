# MuJoCo 학습 튜토리얼 (Tutorial)

MuJoCo 물리 엔진의 핵심 모델링 문법과 파이썬 연동을 단계별로 실습하는 공간입니다.

---

## 🚀 빠른 실행 방법

### 1. 범용 실행 스크립트 (`run.py`) 사용

가상환경(`mujoco_env`)이 활성화된 상태에서 아래와 같이 실행합니다:

```bash
# 기본 예제(01_hello.xml) 뷰어 실행
python run.py

# 특정 번호나 키워드로 실행
python run.py 01
python run.py 02

# XML 파일 전체 이름을 지정하여 실행
python run.py 02_joints_tendon.xml

# (WSL2/서버 등) GUI 없이 터미널에서 물리 연산만 빠르게 검증할 때
python run.py 01 --headless
```

### 2. MuJoCo 내장 뷰어 CLI 직접 사용

```bash
python -m mujoco.viewer --mjcf=01_hello.xml
python -m mujoco.viewer --mjcf=02_joints_tendon.xml
```

---

## 📚 단계별 실습 목차

| 파일명 | 주제 | 핵심 태그 / 개념 |
| :--- | :--- | :--- |
| **`01_hello.xml`** | 기본 세상 구성 및 자유 낙하 | `<worldbody>`, `<light>`, `<geom>` (plane/box/sphere), `<joint type="free"/>` |
| **`02_joints_tendon.xml`** | 다관절 링크 및 텐던(와이어) | `<joint type="ball"/>`, `<joint axis="...">`, `<site>`, `<tendon><spatial>` |

---

## 📝 예제별 핵심 정리

### 01. Hello World (`01_hello.xml`)
- **`<worldbody>`**: 시뮬레이션의 절대 원점(세상 좌표계).
- **`<body>` vs `<geom>`**:
  - `body`: 질량과 위치를 갖는 가상의 뼈대.
  - `geom`: 눈에 보이는 3D 모양과 실제 충돌체.
  - ⚠️ `geom`의 `size`는 **절반 크기(Half-size / 반지름)** 기준입니다.
- **`<joint type="free"/>`**: 6자유도를 부여하여 중력에 따라 자유롭게 낙하 및 회전할 수 있도록 함.

### 02. 다관절 및 텐던 (`02_joints_tendon.xml`)
- **`<joint type="ball"/>`**: 3차원 전방향 회전이 가능한 구체 관절.
- **`<joint axis="0 1 0"/>`**: 특정 축(Y축)을 기준으로만 회전하는 힌지(Hinge) 관절.
- **`<site>`**: 물리적 부피는 없지만 특정 위치를 지정하기 위한 마커 포인트.
- **`<tendon>`**: 사이트(`site`)와 사이트 사이를 연결하여 팽팽하게 당기는 인대/와이어/케이블.

---

## 🎮 MuJoCo 뷰어 조작 단축키

- **`Space`**: 시뮬레이션 일시정지 / 재생
- **`Backspace`**: 시뮬레이션 초기 상태로 리셋
- **`Ctrl + 마우스 우클릭 드래그`**: 물체에 물리적인 힘(Force/Perturbation)을 가해 잡아당기기
- **`마우스 좌클릭 드래그`**: 시점 360도 회전
- **`마우스 휠`**: 시점 확대 / 축소
