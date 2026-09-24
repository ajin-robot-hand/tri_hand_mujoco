# 유체력 및 공기역학 계산 모델 (Fluid Forces)

> 원문 출처: [MuJoCo Documentation - Fluid forces](https://mujoco.readthedocs.io/en/stable/computation/fluid.html)  
> 본 문서는 MuJoCo에서 공기 및 액체와 같은 유체 환경 속 물체의 거동(비행, 유영, 유체 저항 등)을 모사하기 위해 제공하는 **두 가지 유체력 계산 모델(관성 기반 모델 및 타원체 기반 모델)의 수학적 원리, 주요 파라미터, 설정법 및 주의사항**을 정리한 문서입니다.

---

## 1. 유체력 시뮬레이션 개요 (Overview)

### (1) 왜 완전한 전산유체역학(CFD)이 아닌 현상학적 모델인가?
- **Navier-Stokes 방정식의 한계**: 유체의 유동장 전체를 정밀하게 격자망(Grid/Mesh)으로 풀어내는 정통 CFD(전산유체역학)는 연산량이 극도로 높아 로보틱스 제어나 강화학습처럼 빠른 물리 시뮬레이션 환경에는 적합하지 않습니다.
- **무상태 현상학적 모델 (Stateless Phenomenological Model)**:
  - MuJoCo는 유체 공간 자체에 별도의 동적 상태 변수(격자별 속도, 압력 등)를 할당하지 않는 **무상태(stateless)** 방식을 취합니다.
  - 대신 유체 속을 움직이는 강체(Rigid Body)의 기하학적 형태, 속도, 각속도, 가속도로부터 발생하는 **항력(Drag), 양력(Lift), 부가 질량(Added Mass), 점성 마찰(Viscous Resistance)**을 물리 법칙에 기반한 대수적 수식으로 근사 계산합니다.
  - 이 덕분에 연산 비용을 극소화하면서도 곤충/조류의 날갯짓 비행, 물고기의 유영, 수중 낙하 등의 복잡한 거동을 안정적으로 시뮬레이션할 수 있습니다.

### (2) 유체 환경 활성화 조건
유체력 연산은 전역 옵션(`<option>`)에서 유체의 **밀도(`density`, $\rho$)** 또는 **점성(`viscosity`, $\beta$)**을 양수(`> 0`)로 설정하는 즉시 활성화됩니다:
- `density` ($\rho$, 단위: $\text{kg/m}^3$): 유체의 질량 밀도 (속도의 제곱에 비례하는 관성 저항, 양력, 부가 질량에 영향).
- `viscosity` ($\beta$, 단위: $\text{m}^2/\text{s}$): 유체의 동점성 계수 (속도에 선형 비례하는 층류 점성 저항에 영향).
- `wind` (3D 벡터, 단위: $\text{m/s}$): 전역 바람 벡터. 물체의 선속도 계산 시 $(\mathbf{v} - \mathbf{v}_{\text{wind}})$ 형태로 차감되어 유체력이 작용합니다.

> [!TIP]
> **수치 적분기 권장 설정 (`implicit` / `implicitfast`)**  
> 유체력은 물체의 속도에 직접 의존(velocity-dependent)하는 외력입니다. 오일러(Euler)나 Runge-Kutta 계열의 외연적(Explicit) 적분기를 사용하면 속도가 빠를 때 수치적 발산(폭발)이 일어나기 쉽습니다.  
> MuJoCo는 두 유체력 모델 모두에 대해 **해석적 도함수(Analytic Derivatives)**를 완전히 구현해 두었으므로, 유체력을 사용할 때는 반드시 옵션에서 **`integrator="implicit"`** 또는 **`integrator="implicitfast"`**를 사용하는 것이 권장됩니다.

---

## 2. 모델 A: 관성 기반 모델 (Inertia-based Model)

물체에 별도의 기하학적 유체 형상을 지정하지 않았을 때 기본으로 적용되는 모델입니다. 바디(Body)의 **질량과 관성 텐서(Inertia Matrix)**로부터 직육면체 형태의 기하학을 유추하여 유체력을 계산합니다.

### (1) 등가 관성 박스 (Equivalent Inertia Box)
각 바디는 질량 $\mathcal{M}$과 대각 관성 모멘트 $(\mathcal{I}_{xx}, \mathcal{I}_{yy}, \mathcal{I}_{zz})$를 가집니다. MuJoCo는 이 바디와 동일한 질량 및 관성 모멘트를 갖는 가상의 직육면체(반폭, 반깊이, 반높이: $r_x, r_y, r_z$)를 다음과 같이 계산합니다:

$$r_x = \sqrt{\frac{3}{2 \mathcal{M}} \left(\mathcal{I}_{yy} + \mathcal{I}_{zz} - \mathcal{I}_{xx}\right)}$$

$$r_y = \sqrt{\frac{3}{2 \mathcal{M}} \left(\mathcal{I}_{zz} + \mathcal{I}_{xx} - \mathcal{I}_{yy}\right)}$$

$$r_z = \sqrt{\frac{3}{2 \mathcal{M}} \left(\mathcal{I}_{xx} + \mathcal{I}_{yy} - \mathcal{I}_{zz}\right)}$$

이 등가 박스는 바디의 로컬 좌표계에 정렬되어 유체력 계산의 투영 면적으로 사용됩니다.

### (2) 작용하는 힘과 토크의 합
바디 로컬 좌표계에서의 선속도 $\mathbf{v}$와 각속도 $\boldsymbol{\omega}$에 대해 유체가 가하는 총 힘 $\mathbf{f}$와 토크 $\mathbf{g}$는 **이차 항력(Quadratic Drag, 하첨자 $D$)**과 **점성 저항(Viscous Resistance, 하첨자 $V$)**의 합입니다:

$$\mathbf{f}_{\text{inertia}} = \mathbf{f}_D + \mathbf{f}_V$$

$$\mathbf{g}_{\text{inertia}} = \mathbf{g}_D + \mathbf{g}_V$$

#### 1) 이차 항력 (Quadratic Drag: 고 레이놀즈 수)
밀도 $\rho$에 비례하고 속도의 제곱에 비례합니다. 고속 유동(높은 Reynolds 수) 환경의 압력 저항을 근사합니다:
- **힘 성분 ($i$축 방향)**:
  $$f_{D, i} = - 2 \rho \, r_j r_k |v_i| v_i$$
  *(여기서 $r_j r_k$는 진행 방향 $i$축에 수직인 투영 단면의 사분면 면적에 해당)*
- **토크 성분 ($i$축 회전)**:
  $$g_{D, i} = - \frac{1}{2} \rho \, r_i \left(r_j^4 + r_k^4\right) |\omega_i| \omega_i$$
  *(회전 반경에 따른 속도 차이를 표면 전체에 걸쳐 면적 적분한 결과)*

#### 2) 점성 저항 (Viscous Resistance: 저 레이놀즈 수)
점성 계수 $\beta$에 비례하고 속도에 선형 비례합니다. 완만한 유동(낮은 Reynolds 수) 환경에서 등가 구(반지름 $r_{eq} = \frac{r_x + r_y + r_z}{3}$)에 작용하는 스토크스 법칙(Stokes' Law)을 적용합니다:
- **선형 저항력**:
  $$f_{V, i} = - 6 \beta \pi \, r_{eq} v_i$$
- **회전 저항 토크**:
  $$g_{V, i} = - 8 \beta \pi \, r_{eq}^3 \omega_i$$

> [!NOTE]
> 밀도(`density`)를 0으로 두고 점성(`viscosity`)만 양수로 설정하면, 형상에 따른 난류 항력 없이 순수하게 속도에 비례하는 감쇠(damping) 효과만을 시스템 전체에 부여할 수 있습니다.

---

## 3. 모델 B: 타원체 기반 모델 (Ellipsoid-based Model)

단순 박스 근사를 넘어, 날개나 유선형 몸체와 같은 정교한 유체역학 현상(양력, 부가 질량, 와류 순환 등)을 모사하기 위해 도입된 고급 모델입니다. 곤충(예: 초파리 *Drosophila*) 비행 시뮬레이션 연구를 계기로 구현되었습니다.

### (1) 모델 활성화 및 파라미터 구조
- 특정 기하 지오메트리(`<geom>`)에 **`fluidshape="ellipsoid"`** 속성을 부여하면 활성화됩니다.
- **상속 규칙**: 한 지오메트리에 타원체 모델이 켜지면, 그 지오메트리가 속한 부모 바디(parent body)의 관성 기반 유체 모델은 자동으로 비활성화됩니다.
- 지오메트리마다 5개의 무차원 유체 계수 벡터 **`fluidcoef`**를 개별 설정할 수 있습니다:

| 인덱스 | 파라미터 설명 | 기호 | 기본값 (Default) | 물리적 역할 |
| :---: | :--- | :---: | :---: | :--- |
| **0** | 무딘 형상 항력 계수 (Blunt drag) | $C_{D, \text{blunt}}$ | **0.5** | 흐름과 수직인 넓은 투영면에 작용하는 형상 항력 |
| **1** | 유선형/날씬한 항력 계수 (Slender drag) | $C_{D, \text{slender}}$ | **0.25** | 흐름과 평행한 얇은 가장자리/표면 마찰 항력 |
| **2** | 각회전 항력 계수 (Angular drag) | $C_{D, \text{angular}}$ | **1.5** | 회전 시 발생하는 유체 회전 저항 토크 |
| **3** | 쿠타 양력 계수 (Kutta lift) | $C_K$ | **1.0** | 날개 뒷전(trailing edge) 순환으로 발생하는 양력 |
| **4** | 마그누스 양력 계수 (Magnus lift) | $C_M$ | **1.0** | 물체의 자전 회전축과 진행 속도의 외적으로 생기는 횡력 |

### (2) 5가지 작용력 총합
타원체 반경 $\mathbf{r} = (r_x, r_y, r_z)$에 대해 유체가 가하는 총 외력과 토크는 다음 5가지 성분의 합입니다:

$$\mathbf{f}_{\text{ellipsoid}} = \mathbf{f}_A + \mathbf{f}_D + \mathbf{f}_M + \mathbf{f}_K + \mathbf{f}_V$$

$$\mathbf{g}_{\text{ellipsoid}} = \mathbf{g}_A + \mathbf{g}_D + \mathbf{g}_V$$

- $\mathbf{f}_A, \mathbf{g}_A$: 부가 질량력 (Added Mass)
- $\mathbf{f}_D, \mathbf{g}_D$: 점성/형상 항력 (Viscous Drag)
- $\mathbf{f}_M$: 마그누스 양력 (Magnus Lift)
- $\mathbf{f}_K$: 쿠타 양력 (Kutta Lift)
- $\mathbf{f}_V, \mathbf{g}_V$: 선형 점성 저항 (Viscous Resistance)

---

## 4. 타원체 모델의 세부 역학 구성요소

### (1) 타원체 임의 방향 투영 면적 보조정리 (Ellipsoid Projection Lemma)
항력 및 양력 계산 시, 유체 진행 방향 단위 벡터 $\mathbf{u} = (u_x, u_y, u_z)$에 수직인 평면에 타원체(반지름 $r_x, r_y, r_z$)가 투영된 정사영 타원의 면적 $A^{\mathrm{proj}}_{\mathbf{u}}$은 엄밀하게 다음과 같습니다:

$$A^{\mathrm{proj}}_{\mathbf{u}} = \pi \sqrt{\frac{r_y^4 r_z^4 u_x^2 + r_z^4 r_x^4 u_y^2 + r_x^4 r_y^4 u_z^2}{r_y^2 r_z^2 u_x^2 + r_z^2 r_x^2 u_y^2 + r_x^2 r_y^2 u_z^2}}$$

MuJoCo는 임의의 3차원 유동 받음각에서도 이 식을 통해 정확한 단면적을 실시간 계산합니다.

### (2) 부가 질량 (Added Mass / Virtual Mass: $\mathbf{f}_A, \mathbf{g}_A$)
- **개념**: 물체가 유체 속에서 **가속**할 때, 물체뿐 아니라 주변 유체도 함께 밀려나며 가속되어야 합니다. 이때 물체가 느끼는 추가적인 유체 관성 저항입니다.
- **특징**: 포텐셜 유동 이론(Potential Flow)에 기반하므로 점성이 없는 비점성 유체에서도 존재하며, **등속 운동 시에는 0**이 됩니다. 계수 스케일링이 불가능한 순수 기하학적 불변량입니다.
- **수식**:
  $$\mathbf{f}_A = - \mathbf{m}_A \circ \dot{\mathbf{v}} + (\mathbf{m}_A \circ \mathbf{v}) \times \boldsymbol{\omega}$$
  $$\mathbf{g}_A = - \mathbf{I}_A \circ \dot{\boldsymbol{\omega}} + (\mathbf{m}_A \circ \mathbf{v}) \times \mathbf{v} + (\mathbf{I}_A \circ \boldsymbol{\omega}) \times \boldsymbol{\omega}$$
  *(여기서 $\mathbf{m}_A$와 $\mathbf{I}_A$는 Tuckerman(1925)의 타원 적분 계수 $\kappa_i$로부터 유도된 가상 질량 및 관성 모멘트 벡터이며, $\circ$는 요소별 곱)*

### (3) 정교화된 항력 모델 ($\mathbf{f}_D, \mathbf{g}_D$)
날카로운 평판이나 날개는 진행 방향에 따라 수직 투영면(Blunt)과 미끄러지는 평행면(Slender)의 항력 특성이 크게 다릅니다.
- **선형 항력**:
  $$\mathbf{f}_D = - \rho \Big[ C_{D, \text{blunt}} A^{\mathrm{proj}}_{\mathbf{v}} + C_{D, \text{slender}} \left( A_{\text{max}} - A^{\mathrm{proj}}_{\mathbf{v}} \right) \Big] \|\mathbf{v}\| \mathbf{v}$$
  *(여기서 $A_{\text{max}} = \pi r_{\text{max}} r_{\text{mid}}$로 타원체의 최대 단면적)*
- **회전 각항력**:
  타원체가 각 축을 중심으로 회전할 때 쓸고 지나가는 최대 단면 관성 모멘트 $\mathbf{I}_{D, ii} = \frac{8\pi}{15} r_i \max(r_j, r_k)^4$를 계산하여 토크를 도출:
  $$\mathbf{g}_D = - \rho \, \boldsymbol{\omega} \Big( \big[ C_{D, \text{angular}} \mathbf{I}_D + C_{D, \text{slender}} (\mathbf{I}_{\text{max}} - \mathbf{I}_D) \big] \cdot \boldsymbol{\omega} \Big)$$

### (4) 양력 모델 1: 마그누스 효과 (Magnus Force: $\mathbf{f}_M$)
- **개념**: 회전하는 구나 원통이 유체 속을 병진 이동할 때, 표면 마찰에 의해 유체 흐름이 한쪽으로 편향되면서 수직 방향으로 발생하는 힘입니다 (축구공의 바나나킥, 야구공의 변화구 원리).
- **수식**:
  $$\mathbf{f}_M = C_M \, \rho \, V (\boldsymbol{\omega} \times \mathbf{v})$$
  *(여기서 $V = \frac{4}{3}\pi r_x r_y r_z$는 타원체의 부피)*

### (5) 양력 모델 2: 쿠타 조건 (Kutta Condition Lift: $\mathbf{f}_K$)
- **개념**: 날개(Airfoil)나 얇은 판이 유체 속을 비스듬히 나아갈 때, 뾰족한 뒷전(trailing edge)에서 정체점(stagnation point)이 유지되도록 순환(Circulation $\Gamma$)이 유도되어 상방 양력이 발생하는 현상(Kutta-Joukowski 정리)입니다.
- **특징**: 구형 물체($r_x = r_y = r_z$)에서는 대칭성에 의해 $\mathbf{f}_K = 0$이 되며, 납작하거나 날씬한 타원체에서만 발생합니다.
- **3D 확장 수식**:
  $$\mathbf{f}_K = C_K \, \rho \, A^{\mathrm{proj}}_{\mathbf{v}} (\hat{\mathbf{v}} \cdot \hat{\mathbf{n}}_{s, \mathbf{v}}) \Big( (\hat{\mathbf{n}}_{s, \mathbf{v}} \times \mathbf{v}) \times \mathbf{v} \Big)$$
  *(여기서 $\hat{\mathbf{n}}_{s, \mathbf{v}}$는 투영면을 결정하는 타원 단면의 법선 단위 벡터)*

---

## 5. 두 모델 비교 요약

| 비교 항목 | 관성 기반 모델 (Inertia Model) | 타원체 기반 모델 (Ellipsoid Model) |
| :--- | :--- | :--- |
| **적용 단위** | 바디(`<body>`) 단위 자동 적용 | 지오메트리(`<geom>`) 단위 개별 지정 |
| **기하 형상 근사** | 질량/관성 텐서로부터 계산된 등가 박스 | 지오메트리 크기 기반 3축 타원체 |
| **활성화 방법** | `<option density="..." viscosity="..."/>` | `<geom fluidshape="ellipsoid"/>` 추가 지정 |
| **튜닝 파라미터** | 전역 $\rho, \beta$ 2개만 사용 | 전역 $\rho, \beta$ 외 지오메트리당 `fluidcoef` 5개 |
| **부가 질량 ($\mathbf{f}_A$)** | 미지원 (0) | 지원 (Tuckerman 가상 관성 타원 적분) |
| **양력 ($\mathbf{f}_K, \mathbf{f}_M$)** | 미지원 (항력과 선형 저항만 계산) | 지원 (마그누스 효과 + 쿠타 날개 순환 양력) |
| **주요 적용 분야** | 일반 로봇의 단순 공기 저항, 수중 감쇠 효과 | 날갯짓 비행 로봇, 낙하하는 카드/낙엽, 프로펠러 |

---

## 6. tri_hand 프로젝트와의 연계 및 활용 가이드

본 저장소의 3지 다관절 로봇 손(`tri_hand`) 시뮬레이션 환경 관점에서 유체력 모델은 다음과 같은 실무적 의미를 갖습니다:

1. **지상 공기 환경에서의 기본 설정**:
   - 일반적인 로봇 손 조작 시 공기 밀도($\approx 1.2 \, \text{kg/m}^3$)와 점성($\approx 1.5 \times 10^{-5} \, \text{m}^2/\text{s}$)에 의한 공기 저항은 Dynamixel XL430 모터의 토크($1.4 \, \text{N}\cdot\text{m}$) 대비 무시할 수 있을 정도로 작습니다.
   - 따라서 일반 환경에서는 `<option>`의 `density`와 `viscosity`를 0(기본값)으로 유지하여 불필요한 연산 부하를 줄입니다.
2. **수중 조작 또는 점성 유체 속 물체 파지 시뮬레이션**:
   - 로봇 손이 물속(수중 그리퍼)이나 오일 수조 내에서 물체를 조작하는 과제를 수행할 경우, `density="1000"` (물 밀도), `viscosity="0.00089"` 등으로 설정하여 유체 저항과 부가 질량 효과를 시뮬레이션할 수 있습니다.
   - 손가락 링크가 빠르게 오므려질 때 유체 저항력으로 인해 모터에 추가 부하가 걸리는 상황을 사실적으로 재현합니다.
3. **손가락 진동 억제 (Viscous Damping 대체 용도)**:
   - 각 관절마다 기계적 감쇠(`joint damping`)를 일일이 맞추기 어렵거나 물체와의 비접촉 감쇠가 필요할 때, 미세한 전역 `viscosity`를 부여하여 시스템 전체의 고주파 떨림(chattering)을 완화하는 용도로 응용할 수 있습니다.

---

## 7. 초보자가 반드시 겪는 함정과 주의사항 (Clarifications & Pitfalls)

1. **외연적 적분기 사용 시 시뮬레이션 폭발 (Instability with Explicit Integrators)**:
   - 유체력(특히 속도의 제곱에 비례하는 항력 및 가속도에 비례하는 부가 질량)을 켠 상태에서 기본 적분기(`Euler` 등)를 쓰면 고속 회전 시 힘이 급격히 커지며 시뮬레이션이 폭발(NaN 발산)합니다.
   - **해결책**: 반드시 `<option integrator="implicit"/>` 또는 `integrator="implicitfast"`를 지정하세요.
2. **부력(Buoyancy)은 유체력 모델에 포함되지 않음**:
   - 공식 유체력 모델은 **움직임에 의해 발생하는 동적 힘(Dynamic forces: 항력, 양력, 부가 질량)**만을 계산합니다.
   - 정수압에 의한 **정적 부력(Archimedes' Buoyancy: $\rho V g$)은 자동으로 적용되지 않습니다!** 수중 시뮬레이션 시 부력이 필요하다면 중력(`gravity`) 크기를 보정하거나, 각 바디에 수동 외력(`xfrc_applied`)을 인가해야 합니다.
3. **타원체 모델 지정 시 부모 바디의 관성 모델 자동 차단**:
   - 한 링크(Body) 안에 여러 개의 충돌 지오메트리가 있을 때, 그 중 하나에만 `fluidshape="ellipsoid"`를 주면 **해당 부모 바디 전체의 관성 기반 유체력 계산이 꺼집니다.**
   - 따라서 해당 바디의 유체 상호작용을 정밀하게 다루려면 관련 geom들 각각에 적절한 `fluidshape` 및 `fluidcoef`를 명시해주어야 합니다.
4. **STL 메쉬 충돌체와 타원체 모델의 관계**:
   - `AGENTS.md`의 규칙대로 시각 메쉬(`group 1`)와 충돌체 프리미티브(`group 3`)를 분리해 두었을 때, `fluidshape="ellipsoid"`는 메쉬가 아닌 해당 지오메트리의 크기(`size`) 파라미터를 기반으로 3축 반지름($r_x, r_y, r_z$)을 결정합니다.

---

## 8. 권장 체크리스트 (Checklist)

유체 환경 모델링을 적용할 때 다음 항목을 단계별로 점검하세요:

- [ ] `<option>` 태그에 `density` 또는 `viscosity` 값이 양수로 올바르게 선언되었는가?
- [ ] 수치 발산 방지를 위해 `<option integrator="implicit"/>` (또는 `implicitfast`)가 설정되었는가?
- [ ] 바람 효과가 필요한 경우 `<option wind="x y z"/>` 벡터가 월드 좌표계 기준으로 바르게 주어졌는가?
- [ ] 단순 항력 이상의 양력/부가 질량이 필요한 경우, 목표 geom에 `fluidshape="ellipsoid"`를 지정했는가?
- [ ] `fluidcoef` 5개 계수(`[blunt, slender, angular, kutta, magnus]`)의 값이 대상 물체의 특성에 맞게 튜닝되었는가?
- [ ] 수중 시뮬레이션인 경우, 정적 부력(Buoyancy)이 별도로 고려되었는가?
