````markdown
# CARLA RGB Camera Fault Injection Testbench (Fog + Low-Light + Flicker) with UDP Brake Controller

This repository provides a fault-injection testbench for evaluating the robustness of an RGB-camera-based pedestrian sensing and emergency response loop in CARLA. The project injects perception-relevant faults through CARLA weather and lighting controls (fog, night conditions, exposure instability, and optional street-light flicker), publishes telemetry over UDP, and uses an external controller process to issue safety actions (`brake`, `slowdown`, `resume`) based on perceived pedestrian proximity.

The system is split into two Python programs:

- **Simulation node (CARLA + Pygame):** spawns and runs the ego vehicle, configures adverse sensing conditions, renders the front RGB camera, performs a simple pedestrian detection proxy, detects collisions, and streams telemetry to UDP.
- **Controller node (UDP listener):** receives telemetry, applies decision logic, and sends driving commands back to the simulation over UDP.

---

## Key Idea: Fault Injection for RGB Perception

Instead of altering pixel buffers directly, this project injects faults by manipulating environmental and sensor-relevant parameters that degrade the effective information content of the RGB stream. This emulates realistic failure modes for vision systems such as reduced visibility, underexposure, and illumination instability.

### Injected Fault Types

1. **Photometric degradation (low-light / night)**
   - Night-like sun altitude configuration and heavy cloudiness reduce scene brightness and contrast.

2. **Exposure instability (temporal sensor perturbation)**
   - The camera runs in manual exposure mode.
   - `exposure_compensation` is periodically randomized to simulate exposure oscillation or miscalibration.
   - The detection proxy can be configured to drop out under severe underexposure (see Detection Proxy).

3. **Atmospheric degradation (fog scattering / attenuation)**
   - Heavy fog density reduces contrast and effective range.
   - Fog distance is periodically jittered to create time-varying visibility conditions.

4. **Illumination instability (optional street-light flicker)**
   - If supported by the CARLA map and build, a subset of street lights is toggled at a fixed interval.
   - This emulates flickering infrastructure and time-varying illumination.

---

## System Architecture

### UDP Data Flow

- **Simulation → Controller:** telemetry sent to UDP port `9000`
- **Controller → Simulation:** commands sent to UDP port `9001`

The separation allows treating the simulation node as “vehicle + perception under fault injection” and the controller as an external “decision and actuation module.”

---

## Repository Contents

Suggested layout:

```text
.
├── simulation.py   # CARLA simulation, sensors, weather/lighting fault injection, telemetry sender, command receiver
├── controller.py   # UDP controller, decision logic, command sender
└── README.md
````

---

## Requirements

* Python 3.8+
* CARLA server and matching CARLA Python API
* Python packages:

  * `pygame`
  * `numpy`
  * `opencv-python` (imported; not required for the current rendering path)

Install dependencies:

```bash
pip install pygame numpy opencv-python
```

Note: CARLA Python API installation depends on your CARLA setup. Ensure `import carla` works in your environment.

---

## Configuration

### CARLA Host

The simulation script connects means to:

* `localhost:2000` (default CARLA server address/port)

### UDP Ports

* Simulation sends telemetry to: `UDP_PORT_SEND = 9000`
* Simulation receives commands on: `UDP_PORT_RECEIVE = 9001`
* Controller receives telemetry on: `UDP_PORT_RECEIVE = 9000`
* Controller sends commands to: `UDP_PORT_SEND = 9001`

### IP Address Settings 

You must set the destination IPs depending on where you run each process:

#### In `simulation.py`

```python
UDP_IP = "127.0.0.1"  # IP of the controller machine (localhost if same machine)
```

#### In `controller.py`

```python
CARLA_IP = "127.0.0.1"  # IP of the simulation machine (localhost if same machine)
```

If simulation and controller run on different machines:

* `UDP_IP` must be the controller machine IP
* `CARLA_IP` must be the simulation machine IP

---

## Running the Project

### 1) Start CARLA

Start the CARLA server first (example):

```bash
./CarlaUE4.sh
```

### 2) Run the simulation node

In a new terminal:

```bash
python simulation.py
```

This opens a Pygame window showing the front RGB camera and continuously publishes telemetry.

Runtime control:

* Press `A` to respawn the ego vehicle.

### 3) Run the controller node

In another terminal:

```bash
python controller.py
```

The controller prints telemetry and sends control commands back to the simulation.

---

## Fault Injection Details

### Weather-Based Visibility Faults

The simulation configures extreme conditions and updates them during runtime:

* High fog density reduces contrast and range.
* Fog distance is periodically randomized among a small set of meaningfully different values to simulate variable visibility.

These variations cause a time-varying sensing envelope for RGB perception.

### Exposure Perturbation (Sensor-Level Fault)

The camera runs with manual exposure mode, and the simulation periodically randomizes `exposure_compensation`. This simulates:

* exposure oscillation,
* gain/ISO instability,
* challenging transitions in illumination.

This is a common real-world source of transient perception failures.

---


## Safety/Control Loop

### Decision Logic (Controller)

Controller thresholds:

* `BRAKE_RANGE = 6.0` meters
* `SLOWDOWN_RANGE = 10.0` meters

Actions:

* If pedestrian detected at `distance <= BRAKE_RANGE`: send `brake`
* If `BRAKE_RANGE < distance <= SLOWDOWN_RANGE`: send `slowdown`
* After braking, when pedestrian is no longer detected: send `resume`

### Commands

Controller sends JSON payloads:

* `{"cmd": "brake"}`
* `{"cmd": "slowdown", "distance": <float>}`
* `{"cmd": "resume"}`

---

## Telemetry and Metrics

Simulation publishes telemetry including:

* Vehicle speed
* Pedestrian detection flag
* Estimated distance to closest pedestrian in front
* Pedestrian collision flag (from collision sensor)
* Weather state (fog density, fog distance, precipitation)
* Camera state (resolution, FOV, exposure compensation)

### Collision as a Safety Outcome

A collision sensor attached to the ego vehicle flags collisions with pedestrians. This provides a direct outcome measure that can be correlated with injected fault conditions (fog/exposure/flicker) to evaluate failure modes and safety performance.

---





```
```
