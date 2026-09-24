# MuJoCo 솔버 알고리즘, 역역학 및 도함수 (Solvers, Inverse Dynamics & Derivatives)

> 원문 출처: [MuJoCo Documentation - Computation](https://mujoco.readthedocs.io/en/stable/computation/index.html)  
> 본 문서는 MuJoCo의 계산(Computation) 챕터 중 **볼록 최적화 솔버 알고리즘 3종(Newton, CG, PGS), 성능 가속 기법(Warmstart, Islands, Sleeping), 해석적 역역학(`mj_inverse`), 제어/학습을 위한 미분 연산(`mjd_transitionFD`)**을 정리한 제3편(완결편) 문서입니다.

---

## 1. 구속조건 최적화 문제의 수학적 정식화 (Formulation)

MuJoCo는 구속조건力 $f$와 가속도 $\dot{v}$를 구할 때 상호 동치인 두 가지 수학적 문제(원문제와 쌍대문제)를 다룹니다:

### (1) 축소 원문제 (Reduced Primal Problem) — 가속도 $\dot{v}$ 기반
- 변형 슬랙 변수를 해석적으로 소거하여 **가속도 벡터 $x = \dot{v}$에 대한 제약 없는(Unconstrained) 볼록 최적화 문제**로 축소합니다:

$$\dot{v} = \arg \min_x \frac{1}{2} \| x - M^{-1}(\tau - c) \|_M^2 + s\left(J x - a_{\text{ref}}\right)$$

- 여기서 $s(\cdot)$는 볼록하고 1차 연속 미분 가능한(once-continuously-differentiable) **소프트 구속 페널티 함수**입니다.
- 제약 조건이 없는 최적화이므로 **Newton법**이나 **켤레기울기법(CG)**과 같은 고차 수렴 알고리즘을 강력하게 적용할 수 있습니다.

### (2) 쌍대 문제 (Dual Problem) — 구속력 $f$ 기반
- 라그랑주 쌍대성을 취하면 구속조건력 $\lambda = f$에 대한 2차 계획법(QP) 문제가 됩니다:

$$f = \arg \min_\lambda \frac{1}{2} \lambda^T \left( A + R \right) \lambda + \lambda^T \left( a_{\text{unconstrained}} - a_{\text{ref}} \right) \quad \text{subject to} \; \lambda \in \Omega$$

- **$A = J M^{-1} J^T$ (Delassus 연산자)**: 구속조건 공간에서의 역관성 행렬 (대칭 준양의 한정).
- **$R$ (정규화 행렬)**: 소프트 구속조건에 의해 도입된 대각 양의 한정 행렬. $A+R$이 엄밀한 양의 한정(Strictly Positive Definite)이 되도록 보장하여 전역 최소점이 항상 유일하게 존재합니다.
- $\Omega$: 허용 가능한 힘의 볼록 집합 (피라미드 콘은 박스 구속, 타원체 콘은 2차 원뿔 구속 SOCP).

---

## 2. 수치 솔버 알고리즘 3종 비교 (Solvers)

MuJoCo는 옵션(`<option solver="..."/>`)을 통해 3가지 최적화 알고리즘을 제공합니다:

| 솔버 명칭 | 최적화 공간 | 수렴 속도 | 헤시안 갱신 방식 | 주요 특징 및 추천 환경 |
| :---: | :---: | :---: | :---: | :--- |
| **`Newton`**<br>*(기본값, Default)* | 축소 원문제 (가속도 $x$) | **2차 수렴 (매우 빠름)** | 해석적 2계 도함수 + Rank-1 Cholesky 갱신 | **(강력 추천)** 적은 반복 횟수로 극도로 정밀한 해 산출. 로봇 손 조작, 다중 접촉 환경 최적. |
| **`CG`** | 축소 원문제 (가속도 $x$) | 초선형 수렴 (Hager-Zhang) | 행렬 생성 없음 (Matrix-free) | 사전 셋업 비용이 없음. 대규모 접촉이나 신규 `discrete` 적분기와 결합 시 유리. |
| **`PGS`** | 쌍대 문제 (구속력 $\lambda$) | 1차 수렴 (느림) | 1축씩 축별 순차 투영 (Gauss-Seidel) | 고전 게임 엔진 표준 방식. 레거시 모델 호환용. 타원형 콘 처리 시 국소 QCQP 풀이 수행. |

### (1) Newton 솔버 (기본 솔버)의 내부 동작
1. 목적 함수의 1계 미분(Gradient $g$)과 2계 미분(Hessian $H$)을 완전 해석적으로 계산합니다.
2. 제약조건의 상태 전이(비접촉 $\leftrightarrow$ 선형 $\leftrightarrow$ 2차 영역)가 발생할 때 전체 헤시안을 재분해하지 않고, **Rank-1 Cholesky 갱신**을 통해 $O(N^2)$으로 초고속 갱신합니다.
3. 종료 판정 조건: 비용 개선량, 기울기 노름, 그리고 다음 스텝의 예상 비용 감소량인 **뉴턴 감량(Newton decrement: $\frac{1}{2} g^T H^{-1} g$)**이 허용 오차(`tolerance`) 이하로 떨어지면 즉시 종료합니다.

### (2) NoSlip 후처리 패스 (`noslip_iterations`)
- 소프트 접촉 특성상 미세한 표면 미끄러짐(Creep/Slip)이 발생할 수 있습니다.
- 메인 솔버가 수렴한 후 마찰 접선 방향만을 강체($R = 0$)로 가정하고 PGS를 추가 수행하여 정지 마찰 슬립을 완전히 제거하는 기법입니다.
- **주의**: 단일 최적화 문제가 아닌 임시방편(Ad-hoc) 2단계 보정이므로, 복잡한 다중 접촉 환경에서는 수치적 불안정성을 유발할 수 있습니다.

---

## 3. 솔버 연산 가속 아키텍처

### (1) 웜스타트 (Warmstart)
- 이전 타임스텝의 구속력 결과를 폐기하지 않고 현재 스텝의 초기 추정값으로 재사용합니다.
- 이전 힘과 무구속 가속도(`qacc_smooth`)의 비용을 사전 비교하여 더 낮은 쪽을 채택합니다.
- **0회 반복 탈출**: 물체가 가만히 놓여 있는 정지(Quiescent) 상태에서는 웜스타트 지점에서의 쌍대 갭(Duality gap) 증명서가 이미 허용 오차를 만족하므로, **Newton/CG 솔버가 루프를 단 1회도 돌지 않고 즉시 반환**하여 연산 비용이 0에 수렴합니다.

### (2) 구속조건 아일랜드 (Constraint Islands: 블록 대각화)
- 서로 접촉이나 구속으로 연결되지 않은 독립된 기구학 트리들을 분리하여 전체 자코비안 $J$를 **블록 대각화(Block-diagonalization)**합니다.
- 복잡한 씬에서 연산 복잡도가 극적으로 감소하며, 멀티스레딩 병렬 처리가 가능해집니다.

### (3) 수면 모드 (Sleeping Optimization: `sleep="true"`)
- 일정 시간(`mjMINAWAKE`) 동안 속도가 임계값 이하로 유지된 정적 물체 아일랜드를 물리 파이프라인에서 통째로 제외합니다.
- 도미노나 쌓여 있는 블록 더미처럼 대부분의 물체가 멈춰 있는 대규모 환경에서 극적인 프레임레이트 향상을 제공합니다.

---

## 4. 해석적 역역학 (Inverse Dynamics: `mj_inverse`)

역역학은 현재 위치 $q$, 속도 $v$, 그리고 **목표 가속도 $\dot{v}$**가 주어졌을 때 이를 달성하기 위해 필요한 **모터 제어 토크 $\tau$**를 계산하는 과정입니다.

$$\tau = M(q) \dot{v} + c(q, v) - J^T f$$

### (1) 왜 MuJoCo의 역역학은 수치 반복이 필요 없는가?
- LCP 기반 엔진에서는 접촉력이 수치 반복으로만 풀리기 때문에 역역학 계산이 불가능하거나 극도로 어렵습니다.
- 반면 MuJoCo의 쌍대 수식에서는 $A$ 행렬이 완전히 소거되고 **오직 대각 정규화 행렬 $R$만 남습니다**:
  $$f = \arg \min_\lambda \frac{1}{2} \lambda^T R \lambda + \lambda^T (a_{\text{constrained}} - a_{\text{ref}}) \quad \text{s.t.} \; \lambda \in \Omega$$
- 행렬 $R$이 대각선이므로 각 구속조건의 힘이 완전히 독립적으로 분리(Decoupled)되어, **역행렬이나 반복문 없이 닫힌 형태(Closed-form)의 해석적 공식으로 $O(N)$ 시간 안에 즉시 풀립니다.**

### (2) `fwdinv` 일관성 자가 진단 플래그
- 옵션에서 `fwdinv="true"`를 켜면, 매 타임스텝마다 순역학에서 구한 가속도 $\dot{v}$를 역역학 함수 `mj_inverse`에 즉시 통과시켜 원래 토크 $\tau$와의 오차를 계산합니다 (`mjData.solver_fwdinv`).
- 이 오차가 0에 가깝다면 현재 수치 솔버(Newton/CG)가 충분히 전역 최적해로 수렴했음을 물리적으로 검증할 수 있습니다.

---

## 5. 도함수 및 민감도 분석 (Derivatives)

강화학습, 궤적 최적화(Trajectory Optimization), 모델 기반 제어(iLQR, MPC)를 위해 상태 전이 자코비안을 계산하는 기능입니다.

### (1) 수치 미분 유틸리티
- **`mjd_transitionFD`**: 이산 시간 순역학($x_{t+h} = f(x_t, u_t)$)에 대해 $\frac{\partial x_{t+h}}{\partial x_t}$ 및 $\frac{\partial x_{t+h}}{\partial u_t}$ 자코비안을 고속 유한차분(Finite-differencing)으로 산출.
- **`mjd_inverseFD`**: 역역학 토크의 미분 행렬 산출.
- 불필요한 기구학 연산을 건너뛰고 웜스타트와 쿼터니언을 완벽히 보존하여 범용적인 차분 연산 대비 압도적으로 빠릅니다.

> [!CAUTION]
> **접촉력 미분 시 주의사항 (`solimp` 시작점)**  
> MuJoCo의 기본 `solimp` 설정은 표면 접촉 순간 0이 아닌 유한한 최소 임피던스($d_{\min} = 0.9$)에서 시작하므로 수학적으로 접촉 진입 순간 불연속(Stiff gradient)이 발생합니다.  
> 기울기 기반 궤적 최적화나 미분 가능 시뮬레이션을 수행할 때는 접촉력의 진입부가 매끄럽도록 **`solimp`의 $d_{\min}$을 `0`으로 설정**하는 것이 권장됩니다.

---

## 6. tri_hand 프로젝트와의 연계 가이드

1. **솔버 설정 (Newton 솔버 유지)**:
   - 3개의 손가락 링크가 물체를 동시에 쥐는 다중 접촉 폐루프 환경에서는 각 접촉점 간의 상호 간섭이 강합니다.
   - 따라서 1차 수렴에 불과한 `PGS` 대신 헤시안을 고려하는 **`Newton` 솔버(기본값)**를 그대로 유지하는 것이 최적의 파지 안정성을 보장합니다.
2. **`fwdinv`를 활용한 제어기 토크 신뢰성 검증**:
   - `tri_hand`의 Dynamixel 모터 제어 시뮬레이션 중 손가락이 떨리거나 물체를 놓치는 현상이 발생하면, `<option>`에 `fwdinv="true"` 플래그를 켜서 솔버 수렴 오차를 확인하세요. 오차가 크다면 `tolerance`를 낮추거나 `iterations`를 늘려야 합니다.
3. **계산 토크 제어(Computed Torque Control) 구현**:
   - 로봇 손으로 특정 목표 각도 궤적 $q^*(t)$를 정밀 추종하고자 할 때, `mj_inverse` API를 호출하면 원하는 목표 가속도 $\ddot{q}^* + K_p(q^* - q) + K_d(\dot{q}^* - \dot{q})$를 발생시키는 피드포워드 모터 토크를 단 한 번의 함수 호출로 완벽하게 얻을 수 있습니다.

---

## 7. 권장 체크리스트 (Checklist)

솔버 및 고급 동역학 기능을 활용할 때 다음 항목을 점검하세요:

- [ ] 로봇 조작 과제에서 기본 솔버가 `Newton`으로 유지되고 있는가?
- [ ] 정지 마찰 슬립을 억제하기 위해 `noslip`을 적용할 때 복잡한 접촉에서 튕김이 발생하지 않는지 확인했는가?
- [ ] 정지 상태 물체가 많은 대규모 환경에서 `sleep="true"` 플래그를 고려했는가?
- [ ] 순방향 물리 솔버의 수렴 신뢰도를 검증하기 위해 `fwdinv` 진단을 활용했는가?
- [ ] iLQR이나 궤적 최적화 시 `mjd_transitionFD`를 호출하고, 필요 시 `solimp`의 $d_{\min}=0$으로 부드러운 접촉 미분을 유도했는가?
