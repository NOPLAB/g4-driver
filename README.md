# g4-driver

[![CI](https://github.com/NOPLAB/g4-driver/actions/workflows/ci.yml/badge.svg)](https://github.com/NOPLAB/g4-driver/actions/workflows/ci.yml)

BLDC Driver with STSPIN32G4

## Simulation Results

FOC制御シミュレーションの結果です。`bldc-sim`クレートで生成されます。

### Step Response (500 RPM)

![Step Response 500 RPM](docs/images/simulation/step_response_500rpm.png)

### Load Disturbance Response

![Load Disturbance](docs/images/simulation/load_disturbance_small.png)

### Startup Response

![Startup](docs/images/simulation/startup_basic.png)

### Ramp Response

![Ramp Response](docs/images/simulation/ramp_0_to_500.png)

### Simulation Parameters

#### Motor Parameters (Small BLDC ~100W)

| Parameter | Value | Unit | Description |
|-----------|-------|------|-------------|
| R_s | 0.5 | Ω | Stator resistance per phase |
| L_d | 0.5 | mH | d-axis inductance |
| L_q | 0.5 | mH | q-axis inductance |
| λ_m | 10 | mWb | PM flux linkage |
| J | 10 | g·cm² | Rotor inertia |
| B | 0.00001 | N·m·s/rad | Viscous friction |
| Pole pairs | 6 | - | Number of pole pairs |
| V_dc | 24 | V | DC bus voltage |
| I_max | 10 | A | Maximum phase current |

#### FOC Controller Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Kp | 0.5 | Speed PI proportional gain |
| Ki | 0.05 | Speed PI integral gain |
| Max acceleration | 500 RPM/s | Speed ramp rate limit |
| Control frequency | 2.5 kHz | FOC update rate |

#### Simulation Settings

| Parameter | Value | Description |
|-----------|-------|-------------|
| Physics dt | 10 μs | Integration time step |
| Control period | 400 μs | FOC control loop period |
| Integration method | RK4 | Runge-Kutta 4th order |

## Documentation

- [STSPIN32G4 Datasheet (PDF)](https://www.st.com/resource/en/datasheet/stspin32g4.pdf)
- [STSPIN32G4 Product Page](https://www.st.com/en/motor-drivers/stspin32g4.html)
