# Code Architecture

## State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> HOVER_TEST : short press < 1s
    IDLE --> ARMING     : long press ≥ 1s

    HOVER_TEST --> IDLE : press again (disarms)

    ARMING --> SPRINTING : 1500ms settle elapsed

    SPRINTING --> HOLDING : alt ≥ SPRINT_CUTOFF_M\nautorotation motor starts
    SPRINTING --> CUT     : safety — missionTime ≥ 8s

    HOLDING --> PUNCHING : missionTime ≥ PUNCH_START_MS
    HOLDING --> CUT      : safety — missionTime ≥ 8s

    PUNCHING --> CUT : missionTime ≥ 8s

    CUT --> DONE : FC disarmed, motors off
```

---

## Variable & Data Flow

```mermaid
flowchart TD
    subgraph BLE ["BLE  NimBLE — 'Quad-Tuner'"]
        direction TB
        W1["CBu16.onWrite()\nHOVER_UUID\nSPRINT_THROT_UUID\nPUNCH_THROT_UUID"]
        W2["CBfloat.onWrite()\nSPRINT_CUTOFF_UUID  ÷100\nHOLD_KP_UUID  ÷10"]
        W3["CBu32.onWrite()\nPUNCH_START_UUID"]
    end

    subgraph PARAMS ["Tunable Parameters  volatile"]
        direction TB
        HT["HOVER_THROTTLE\ndefault 1420 µs"]
        ST["SPRINT_THROTTLE\ndefault 1850 µs"]
        SC["SPRINT_CUTOFF_M\ndefault 17.0 m"]
        KP["HOLD_KP\ndefault 120.0"]
        PS["PUNCH_START_MS\ndefault 7500 ms"]
        PT["PUNCH_THROTTLE\ndefault 2000 µs"]
    end

    subgraph SENSOR ["Sensor  UART1 MSP"]
        ALT["getAltitude()\nMSP_ALTITUDE req → FC\nreturns alt_cm ÷ 100"]
    end

    subgraph SM ["State Machine  50 Hz loop"]
        direction TB
        SPR["SPRINTING\nchannels[2] = SPRINT_THROTTLE"]
        HLD["HOLDING\nholdThrottle(alt)\n= HOVER + KP × error"]
        PUN["PUNCHING\nchannels[2] = PUNCH_THROTTLE"]
        CUT["CUT\nchannels[4] = 1000  disarm\nchannels[2] = 1000"]
        HVT["HOVER_TEST\nchannels[2] = HOVER_THROTTLE"]
    end

    subgraph OUT ["Outputs"]
        MSP["sendRC()\nMSP_SET_RAW_RC\n→ FC via UART1"]
        PWM["ledcWrite GPIO6\n→ MOSFET → brushed motor"]
        LED["digitalWrite GPIO8\nstatus LED pattern"]
    end

    W1 -->|writes| HT
    W1 -->|writes| ST
    W1 -->|writes| PT
    W2 -->|writes| SC
    W2 -->|writes| KP
    W3 -->|writes| PS

    ALT -->|altitude m| HLD
    ALT -->|altitude m| SPR

    HT --> HLD
    KP --> HLD
    ST --> SPR
    SC --> SPR
    PS --> HLD
    PT --> PUN
    HT --> HVT

    SPR --> MSP
    HLD --> MSP
    PUN --> MSP
    CUT --> MSP
    HVT --> MSP

    SPR -->|"prespunUp=true\non HOLDING entry"| PWM
    OUT --> LED
```
