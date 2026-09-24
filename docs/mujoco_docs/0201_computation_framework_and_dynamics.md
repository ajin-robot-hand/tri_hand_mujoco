# MuJoCo 계산 프레임워크와 동역학 (Computation Framework & Dynamics)

> 원문 출처: [MuJoCo Documentation - Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)  
> 본 문서는 MuJoCo의 공식 계산(Computation) 문서 중 **일반화 좌표계 운동방정식, 기구학/역학 파이프라인, 액추에이터 제어 모델, 수치 적분기 5종의 수학적 원리와 실무 선택 기준**을 집중적으로 정리한 제1편 문서입니다.

---

## 1. MuJoCo 계산 파이프라인 개요

MuJoCo는 연속 시간(Continuous Time) 상에서 다체 동역학(Multi-body Dynamics)을 계산하고, 이를 지정된 타임스텝($h = \text{timestep}$) 동안 수치 적분하여 다음 시점의 상태를 갱신합니다.

```
[qpos(t), qvel(t)] (현재 상태)
       │
       ▼
 1. 순기구학 (Forward Kinematics) ─── 글로벌 링크 위치/자세, 관절 축, 서브트리 질량중심 계산
       │
       ▼
 2. 비구속 힘 및 관성 계산 ────────── c(q,v) [RNE 알고리즘] / M(q) [CRB 알고리즘]
       │                              패시브 힘(스프링/댐퍼/중력) + 액추에이터 제어력
       │
       ▼
 3. 충돌 검출 및 구속조건 생성 ────── 점 접촉, 관절 한계, 등가 구속조건 (Jacobian J 생성)
       │
       ▼
 4. 구속조건 최적화 솔버 ─────────── 제약력 f 계산 (PGS / CG / Newton)
       │
       ▼
 5. 순가속도(qacc) 산출 ──────────── a = M⁻¹ (τ + Jᵀf - c)
       │
       ▼
 6. 수치 적분기 (Integrator) ──────── [qpos(t+h), qvel(t+h)] 갱신
```

---

## 2. 일반화 좌표계와 연속시간 운동방정식

### (1) 핵심 표기법 및 MuJoCo 데이터 구조 대응

| 수학 기호 | 크기(차원) | 물리적 의미 | `mjModel` / `mjData` 필드 |
| :---: | :---: | :--- | :--- |
| $\nq$ | 스칼라 | 일반화 위치 좌표 차원 | `mjModel.nq` |
| $\nv$ | 스칼라 | 시스템의 자유도 (자유 속도 차원) | `mjModel.nv` |
| $\nc$ | 스칼라 | 활성화된 구속조건(Constraint) 개수 | `mjData.nefc` |
| $q$ | $\nq$ | 관절 위치 벡터 | `mjData.qpos` |
| $v$ | $\nv$ | 관절 속도 벡터 | `mjData.qvel` |
| $\dot{v}$ ($a$) | $\nv$ | 관절 가속도 벡터 | `mjData.qacc` |
| $\tau$ | $\nv$ | 인가된 총 일반화 힘 (패시브 + 액추에이터 + 외력) | `qfrc_passive + qfrc_actuator + qfrc_applied` |
| $c(q, v)$ | $\nv$ | 바이어스 힘 (코리올리 + 원심력 + 중력) | `mjData.qfrc_bias` |
| $M(q)$ | $\nv \times \nv$ | 관절 공간 관성 행렬 (대칭 양의 한정) | `mjData.qM` |
| $J(q)$ | $\nc \times \nv$ | 구속조건 자코비안 (Jacobian) | `mjData.efc_J` |
| $f$ | $\nc$ | 구속조건에 의해 가해지는 접촉력/제약력 | `mjData.efc_force` |

> [!IMPORTANT]
> **왜 $\nq > \nv$ 인가? (위치 차원과 속도 차원의 차이)**  
> 3차원 공간에서 회전을 자유롭게 표현하기 위해 볼 조인트(Ball joint)나 자유 관절(Free joint)은 짐벌락이 없는 **4차원 쿼터니언(Quaternion, 단위 구 $SO(3)$)**을 사용합니다.  
> 쿼터니언의 위치 차원은 4이지만, 회전 각속도는 3차원 탄젠트 공간에 존재하므로 위치 차원(`nq`)이 자유도(`nv`)보다 커집니다.  
> 따라서 쿼터니언이 포함된 모델에서는 단순 $q_{t+h} = q_t + h v$ 덧셈이 성립하지 않으며, MuJoCo 내부의 쿼터니언 적분 또는 `mj_differentiatePos()` 함수를 통해 차이를 계산해야 합니다.

### (2) 기본 운동방정식 (Equation of Motion)
연속 시간에서의 다체 시스템 운동방정식은 다음과 같이 기술됩니다:

$$M(q) \dot{v} + c(q, v) = \tau + J(q)^T f$$

관성 행렬 $M(q)$는 항상 역행렬이 존재(Symmetric Positive Definite)하므로, 순역학(Forward Dynamics)과 역역학(Inverse Dynamics)은 다음과 같이 명쾌하게 성립합니다:
- **순역학 (Forward Dynamics)**:
  $$\dot{v} = M^{-1} \left(\tau + J^T f - c\right)$$
- **역역학 (Inverse Dynamics)**:
  $$\tau = M \dot{v} + c - J^T f$$

### (3) 알고리즘적 연산 최적화
- **바이어스 힘 $c(q,v)$ 계산**: 가속도를 0으로 둔 상태에서 **RNE(Recursive Newton-Euler)** 알고리즘을 수행하여 $O(N)$의 선형 복잡도로 신속히 계산합니다.
- **관성 행렬 $M(q)$ 계산**: **CRB(Composite Rigid-Body)** 알고리즘을 사용합니다. 기구학 트리 구조 특성상 $M$은 고도로 희소(sparse)하므로, MuJoCo는 이를 전용 희소 행렬 포맷으로 저장하고 $L^T D L$ 분해를 적용해 메모리와 연산량을 획기적으로 줄입니다.

---

## 3. 액추에이션 모델 (Actuation Model)

MuJoCo의 액추에이터는 제어 입력 $u$가 관절 토크로 변환되는 과정을 **전송(Transmission), 활성화 동역학(Activation Dynamics), 힘 생성(Force Generation)** 3단계로 명확히 분리하여 처리합니다.

### (1) 전송 메커니즘 (Transmission)
액추에이터의 스칼라 힘 $p_k$가 어떤 경로를 통해 기구학적 관절력으로 전달되는지 정의합니다:
- `joint`: 지정된 관절에 직접 토크/힘 인가.
- `tendon`: 공간 텐던을 당겨 여러 관절을 동시에 연동 구동 (와이어 구동 로봇).
- `site`: 지정된 3D 좌표 사이트 프레임에 데카르트 힘/토크 인가 (추진체, 엔드이펙터 외력).
- `slider-crank`: 크랭크-슬라이더 기구를 수학적으로 매핑하여 피스톤 왕복 운동을 회전력으로 변환.
- `body`: 물체 표면 접촉점에 흡착력 인가 (진공 그리퍼, 점착 패드).

액추에이터의 길이 $l_k(q)$에 대해 모멘트 암 벡터 $\nabla l_k(q)$가 생성되며, 총 액추에이터 관절 토크는 다음과 같이 합산됩니다:

$$\tau_{\text{actuator}} = \sum_k \nabla l_k(q) \, p_k \quad \longrightarrow \quad \texttt{mjData.qfrc\_actuator}$$

### (2) 활성화 동역학 (Stateful Actuators)
공압/유압 실린더나 생체 근육처럼 즉각적으로 힘이 나오지 않고 내부 압력/흥분 상태가 서서히 차오르는 3차 동역학 액추에이터를 지원합니다.
- `none`: 내부 상태 없음 (일반 DC 모터 등 2차 시스템).
- `integrator`: $\dot{w} = u$
- `filter`: $\dot{w} = (u - w) / t_{\text{const}}$ (1차 지연 필터, 오일러 적분).
- `filterexact`: 해석적 지수 적분($w_{t+h} = w_t + (u - w)(1 - e^{-h/t})$)을 적용하여 타임스텝이 필터 시상수보다 커도 수치 발산하지 않음.
- `muscle`: 생체 근육 특유의 비선형 활성화 수식 적용.

### (3) 힘 생성 공식 (Force Generation)
단일 입력-단일 출력(SISO) 액추에이터의 스칼라 힘 $p_i$는 제어값 $u_i$(또는 활성화 $w_i$), 현재 액추에이터 길이 $l_i$, 속도 $\dot{l}_i$에 대해 다음과 같은 **아핀(Affine) 선형 결합**으로 결정됩니다:

$$p_i = \left(a \, u_i \text{ 또는 } a \, w_i\right) + b_0 + b_1 l_i + b_2 \dot{l}_i$$

- **게인(Gain) $a$** (`actuator_gainprm`): 제어 입력에 곱해지는 배수.
- **바이어스(Bias) $b_0, b_1, b_2$** (`actuator_biasprm`): 길이 및 속도 피드백 항.

> [!TIP]
> **로보틱스 위치/속도 서보와의 매핑 원리 (`tri_hand` 예시)**  
> 로보틱스의 전형적인 PD 위치 제어기 수식은 $\tau = K_p (u - q) - K_v \dot{q}$ 입니다.  
> 이를 MuJoCo의 아핀 공식 $p = a u + b_0 + b_1 l + b_2 \dot{l}$과 비교하면:
> - $a = K_p$ (Gain)
> - $b_0 = 0$
> - $b_1 = -K_p$ (Position Bias)
> - $b_2 = -K_v$ (Velocity Bias)  
> 즉, MuJoCo의 `<position kp="12" kv="0.5"/>` 단축 표기는 내부적으로 이 아핀 파라미터들을 자동으로 채워 넣는 구조입니다.

---

## 4. 패시브 힘 (Passive Forces)

제어 입력이나 구속 가속도와 무관하게 **오직 현재 위치 $q$와 속도 $v$에만 의존하는 자연계의 힘**입니다 (`mjData.qfrc_passive`). 순역학과 역역학 계산 모두에 동일하게 입력값으로 사용됩니다.

1. **관절 및 텐던의 스프링-댐퍼**:
   - 선형 탄성 복원력($-k(q - q_{\text{ref}})$) 및 선형 점성 감쇠($-b v$).
2. **다항식 비선형 힘 (Polynomial Forces)**:
   - 고차 다항식을 통한 비선형 강성 및 감쇠 모델링 지원.
   - **반대칭화 (Anti-symmetrization)**: 짝수 차수 항을 $v|v|$ 형태로 표현하여 속도 반전 시 힘의 방향이 항상 정반대로 뒤집히도록 보장 ($f(-v) = -f(v)$).
   - **부호 보존 (Sign-preservation)**: $z \cdot f(z) \ge 0$ 조건을 만족해야 에너지가 자발적으로 생성되어 시스템이 발산하는 현상을 막을 수 있습니다.
3. **중력 보상 (`gravcomp`)**: 바디 단위로 중력의 일부 또는 전부를 상쇄하는 부력/지지력 인가.
4. **유체력 (Fluid forces)**: 주변 매질에 의한 항력 및 점성 저항.

---

## 5. 수치 적분기 5종 상세 비교 (Numerical Integrators)

연속시간 가속도 $\dot{v}$가 구해지면, 타임스텝 $h$ 동안 상태를 전진시키는 수치 적분기를 실행합니다.

```
       [속도 갱신식]                        [적분기 타입]
v(t+h) = v(t) + h · a(t)          ─── Semi-implicit (Euler)
v(t+h) = v(t) + h · a(t+h)        ─── Implicit-in-velocity (implicit / implicitfast)
v(t+h) = v(t) + h · a_discrete    ─── Discrete-time (discrete)
```

MuJoCo는 속도-암시적 적분을 위해 다음과 같은 **유효 관성 행렬 $\widehat{M}$**을 사용합니다:

$$v_{t+h} = v_t + h \widehat{M}^{-1} M a(v_t), \qquad \widehat{M} \equiv M + h D$$

*(여기서 $D \equiv -\frac{\partial (\tau - c + J^T f)}{\partial v}$ 는 힘의 속도 편미분 행렬)*

### 적분기별 특성 요약

| 적분기 | `integrator` 명칭 | 유효 감쇠 $D$ 포함 범위 | 행렬 분해 방식 | 주요 특징 및 권장 용도 |
| :--- | :---: | :--- | :---: | :--- |
| **반외연적 오일러** | `Euler` | 관절 댐핑(`damping`)만 포함 (대각선) | Cholesky ($L^TL$) | 과거 레거시 모델 호환용. 감쇠가 크면 진동 발생 쉬움. |
| **고속 속도-암시적** | `implicitfast` | 유체력, 텐던 댐핑, 액추에이터 피드백 등 (RNE 제외 후 대칭화) | Cholesky ($L^TL$) | **(강력 추천)** 계산 비용은 Euler와 유사하면서 수치 안정성이 월등함. 대부분의 로봇 모델 기본값. |
| **완전 속도-암시적** | `implicit` | RNE 코리올리/원심력 미분까지 완전 포함 (비대칭) | 희소 $LU$ 분해 (No fill-in) | 고속 회전하는 다관절 펜듈럼 등 강한 코리올리 연성 시스템에 최적. |
| **이산 시간 적분기** | `discrete` | $D$(감쇠)뿐 아니라 $h^2 K$(위치 강성)까지 구속조건과 동시 해석 | 메트릭 통합 반복 솔버 | **(최신 2026.09 기능)** 극도로 뻣뻣한 스프링/서보에서도 폭발하지 않는 무조건 안정성 제공. |
| **4차 룬게-쿠타** | `RK4` | 4단계 다중 스텝 적분 (외연적) | 없음 | 비감쇠 에너지 보존계(무마찰 진자 등)의 장기 궤적 정밀 적분용. |

---

## 6. tri_hand 프로젝트와의 연계 및 실무 권장 설정

1. **적분기 선택 (`implicitfast` 채택)**:
   - `tri_hand`는 Dynamixel 모터의 위치 제어(`kp="12"`, `kv="0.5"`)로 인해 속도 의존성 D 게인이 걸려 있습니다.
   - 기본 `Euler` 적분기보다 **`integrator="implicitfast"`**를 사용하는 것이 관절 떨림(chattering) 억제와 시뮬레이션 안정성 확보에 훨씬 유리합니다.
2. **관절 회전축 및 자유도 구성**:
   - `tri_hand`의 손가락은 모두 단일 회전 관절(`hinge`, 축: `+X`)이므로 각 관절당 `nq = 1`, `nv = 1`입니다.
   - 쿼터니언이 관여하지 않으므로 $q$와 $v$의 차원이 1:1로 일치하여 직관적인 각도(rad) 및 각속도(rad/s) 분석이 가능합니다.
3. **바이어스 힘과 중력 방향 (-Y)**:
   - 본 프로젝트는 `gravity="0 -9.81 0"`(-Y축)이므로, RNE 알고리즘이 계산하는 바이어스 힘 벡터 $c(q, v)$에는 -Y 방향 중력 가속도에 의한 링크 자중 모멘트가 반영됩니다.

---

## 7. 권장 체크리스트 (Checklist)

모델의 동역학 및 수치 적분을 설정할 때 다음 항목을 점검하세요:

- [ ] 액추에이터의 위치 제어 게인(`kp`)과 속도 댐핑(`kv`)이 발산하지 않는 현실적인 값인가?
- [ ] 서보 모터나 유체 저항을 포함한 모델에서 `integrator="implicitfast"`(또는 `implicit`)를 지정했는가?
- [ ] 고강성 스프링이나 극단적인 위치 제어 게인을 쓸 때 `discrete` 적분기를 검토했는가?
- [ ] 에너지 보존이 중요한 자유 진동 계에서 `RK4`를 사용하고 감쇠 요소를 제거했는가?
- [ ] 쿼터니언을 포함하는 자유 관절(Free body) 사용 시 위치 갱신 연산에 단순 덧셈이 아닌 전용 유틸리티(`mj_differentiatePos`)를 사용했는가?
