# Motor Alignment and Hall Sensor Calibration Data

## Test 1: Static Alignment (Two Wires Energized, One Floating)
For this test, two motor phase wires are connected to a DC voltage source ($24\text{ V}, 0\text{ V}$) and one wire is kept floating ($f$). We monitor the resulting stable Hall sensor outputs ($h1$, $h2$, $h3$) at each vector alignment.

| Red Wire | Yellow Wire | Black Wire | h1 | h2 | h3 |
| :---: | :---: | :---: | :---: | :---: | :---: |
| 24 | f | 0 | 1 | 0 | 1 |
| 0 | f | 24 | 0 | 1 | 0 |
| 24 | 0 | f | 1 | 0 | 0 |
| 0 | 24 | f | 0 | 1 | 1 |
| f | 24 | 0 | 0 | 0 | 1 |
| f | 0 | 24 | 1 | 1 | 0 |

---

## Test 2: Manual Rotation (Hall Sensor Sequence)
For this test, we disconnect the motor from the power supply. We spin/turn the motor manually and log the consecutive transitions of the Hall sensors to capture the correct commutation sequence.

| h1 | h2 | h3 |
| :---: | :---: | :---: |
| 0 | 0 | 1 |
| 0 | 1 | 1 |
| 0 | 1 | 0 |
| 1 | 1 | 0 |
| 1 | 0 | 0 |
| 1 | 0 | 1 |

---

## Hardware Connection Mapping
The phase-to-timer associations on the STM32F446RE are mapped as follows:
* **Red Wire:** Connected to Leg 1 (`TIM1_CH1`)
* **Yellow Wire:** Connected to Leg 2 (`TIM1_CH2`)
* **Black Wire:** Connected to Leg 3 (`TIM1_CH3`)