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

## Documentation

- [STSPIN32G4 Datasheet (PDF)](https://www.st.com/resource/en/datasheet/stspin32g4.pdf)
- [STSPIN32G4 Product Page](https://www.st.com/en/motor-drivers/stspin32g4.html)
