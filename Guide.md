# Hardware-in-the-Loop (HITL) Simulation

Comprehensive technical documentation for the architecture, configuration, and sequential deployment steps for running an autonomous internal Hardware-in-the-Loop simulation using ArduPilot's SimOnHardware firmware architecture — without an external physics engine (e.g., Gazebo or AirSim).

The target system uses an integrated ground control station (MK12 Handheld Transmitter), MAVProxy as a serial data multiplexer, and ROS (`mavros`) for high-level offboard autonomous logic.

---

## Chapter 1: Architectural Workflow

In a standard HITL loop, sensor generation and flight dynamics are offloaded to an external simulator. In a **Zero-Engine HITL (SimOnHardware)** setup, the mathematical kinematic models are compiled directly into the microcontroller alongside the autopilot core logic.

```
                SimOnHardware
            ┌──────────────────────────────────────┐
            │      Flight Controller Hardware      │
            │                                      │
            │   ┌──────────────┐   Virtual IMU/Mag │
            │   │  Kinematic   │────────────────┐  │
            │   │Physics Model │                │  │
            │   └──────────────┘                ▼  │
            │          ▲                 ┌────────┐│
            │          │                 │ Ardu-  ││
            │          └─────────────────│ Pilot  ││
            │            Actuator PWM    │  Core  ││
            │                            └────────┘│
            └─────────────────────────────────┬────┘
                                  │ MAVLink Telemetry
                                  ▼ (via USB: /dev/ttyACM*)
                            ┌───────────┐
                            │ MAVProxy  │
                            └─────┬─────┘
                                  │
        ┌─────────────────────────┴─────────────────────────┐
        ▼                                                   ▼
UDP Port (e.g., 14550)                          UDP Port (e.g., 14551)
  ┌───────────────┐                               ┌───────────────┐
  │ QGroundControl│                               │   ROS Node    │
  │  (on MK12)    │                               │   (`mavros`)  │
  └───────────────┘                               └───────────────┘
```

**Internal Kinematics:** The flight controller captures its own generated actuator commands, feeds them into an internal physics equation loop, and pushes synthetic sensor data directly into the Extended Kalman Filter (EKF).

**Multiplexing via MAVProxy:** MAVProxy opens the physical USB interface (`/dev/ttyACM*`), reads the unified stream, and splits it out to multiple virtual local ports.

**Execution Endpoint:** Ground control visualization (QGC) and high-level autonomous override loops (ROS) receive identical telemetry frames without competing for the same raw hardware descriptor.

---

## Chapter 1.2: Firmware Acquisition & Architectural Workflow

Before configuring parameters, you must flash the target hardware with a specialized simulation binary. Standard production ArduCopter firmware builds look for physical SPI/I2C sensor buses on boot. The **SimOnHardware** binary compiles ArduPilot's internal kinematic simulation physics engine directly alongside the core flight controller logic on the microcontroller chip.

### 1. Downloading the Correct Firmware Build
Navigate to the official ArduPilot firmware server to find your specific hardware target (e.g., Pixhawk4, CubeOrange, MatekH743):

1. Go to the ArduPilot Firmware repository: `https://firmware.ardupilot.org/Copter/latest/`
2. Search for your specific board directory suffix appended with **`-sim`** (e.g., `CubeOrange-sim` or `Pixhawk4-sim`). 
3. Download the `arducopter.apj` firmware file from that specific simulation directory.

> ⚠️ **NOTE:** If a pre-compiled `-sim` binary is not available for your specific board target on the server, you must compile it from the ArduPilot source tree using the following terminal commands:
> ```bash
> ./waf configure --board your_board_name--sim
> ./waf copter
>

## Chapter 2: Hardware Parameter Configuration

> **Warning:** Before changing parameters, ensure your physical sensors are safe. Moving to a simulation firmware shares parameters with your real flight settings. **Back up your production configurations (`.param` files) before proceeding.**

Connect your board to your Ground Control Station and configure the following parameters sequentially.

### 1. Enable Hardware Simulation Mapping

| Parameter | Value | Description |
|---|---|---|
| `GPS_TYPE` | `200` | Bypasses hardware UART parsing for GNSS receivers and locks the EKF into accepting internally injected MAVLink `HIL_GPS` position packets. |
| `SIM_OH_MASK` | `15` | (Simulation On Hardware Output Mask). Setting this to `15` (binary `1111`) opens physical PWM output channels 1–4 to behave as virtual actuators. |

### 2. Configure Local Navigation Target Sources (EKF3)

Tell the Kalman Filter to map its coordinate estimators to virtual inputs rather than physical hardware:

| Parameter | Value | Description |
|---|---|---|
| `AHRS_EKF_TYPE` | `3` | Enables EKF3 execution. |
| `EK3_SRC1_POSXY` | `3` | Binds horizontal position mapping to Virtual GPS. |
| `EK3_SRC1_VELXY` | `3` | Binds horizontal velocity to Virtual GPS. |
| `EK3_SRC1_POSZ` | `1` | Uses internal virtual barometer tracking for altitude data. |

### 3. Establish Origin Point (Optional Calibration)

If your simulated vehicle drops out of bounds or displays at global coordinates `(0, 0)`, initialize its default origin coordinates on boot:

| Parameter | Example Value | Description |
|---|---|---|
| `SIM_OPOS_LAT` | `20.2458` | Target latitude. |
| `SIM_OPOS_LNG` | `86.4521` | Target longitude. |
| `SIM_OPOS_ALT` | *(meters)* | Target altitude above mean sea level (meters). |

---

## Chapter 3: Data Bridge Deployment (MAVProxy)

Because you are using an MK12 integrated station, the hardware maps directly as a physical device node via USB. MAVProxy must take ownership of this node and bind it to unique pipelines for the software layer.

Execute the core MAVProxy bridge using the following command. Ensure you map the exact identifier of your device string:

```bash
mavproxy.py --master=/dev/ttyACM0 --baud=115200 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
```

**Parameter Breakdown:**

| Flag | Value | Purpose |
|---|---|---|
| `--master` | `/dev/ttyACM0` | Attaches the proxy layer directly to the physical USB telemetry endpoint exposed by the flight controller. |
| `--baud` | `115200` | Synchronizes serial communication timing blocks. |
| `--out` | `udp:127.0.0.1:14550` | Creates a dedicated output loopback stream for Ground Control software. QGroundControl automatically polls UDP port `14550` locally on your MK12 transmitter. |
| `--out` | `udp:127.0.0.1:14551` | Sets up an isolated data stream channel dedicated to low-latency processing over ROS. |

---

## Chapter 4: Robot Operating System (ROS) Execution

With MAVProxy actively managing the hardware link and providing clean ports, you can launch your communication node without serial block errors.

### 1. Launch MAVROS Node

Direct your MAVROS execution scripts to the target UDP pipeline generated in Chapter 3:

```bash
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@
```

### 2. Verify Simulation Pipeline Status

To confirm that your ROS environment is communicating with the internal hardware simulator without data loss, verify state transmission in a separate terminal:

```bash
rostopic echo /mavros/state
```

Look for the following values to verify success:

| Field | Expected Value |
|---|---|
| `connected` | `True` |
| `guided` | `True` *(if offboard target mode handles execution patterns)* |
| `mode` | `"GUIDED"` or `"STABILIZE"` *(reflecting your radio transmitter switch parameters)* |

### 3. Read Local State Estimates

Verify that the internal physics calculations are outputting valid telemetry to your workspace topics:

```bash
rostopic echo /mavros/local_position/pose
```

You should see stable coordinate telemetry targets without pre-arm failures such as `Bad Variance` or `No GPS Lock`. This confirms that your internal hardware setup is correctly calculating physics equations on the fly.
