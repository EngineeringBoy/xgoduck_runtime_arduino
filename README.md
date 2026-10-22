# XGO Duck

This project runs walking, get-up, pick, and roulade policies on an XGODuck built around the Arduino Uno Q.

It is based on [Microduck](https://github.com/pollen-robotics/microduck) by [Pollen Robotics](https://pollen-robotics.com/microduck/).

The policies here follow that project: 15 servos, a 50 Hz neural controller, and the same joint order.

## What runs where

The Uno Q is two computers.

| | Host | MCU |
| --- | --- | --- |
| Silicon | Qualcomm Linux | STM32U585, Zephyr |
| Rate | 50 Hz policy | 100 Hz bus loop |
| Work | ONNX Runtime, one thread; web UI on port 9527 | 15-servo bus, QMI8658, Bridge report thread |
| Link | Official Arduino App container | `Bridge` at 2 Mbps |

The MCU loop does not wait for Linux. It publishes a 240-byte `duck_state` snapshot at 100 Hz. The host filters each snapshot on arrival and the 50 Hz thread reads the latest one. A late cycle is skipped. There is no queued backlog of actions.

```text
browser :9527
    |
Linux  50 Hz ONNX  ---- duck_command (80 bytes) ---->  STM32 100 Hz
    ^                                                      |
    +------------- duck_state (240 bytes) -----------------+
                                                           |
                                              Serial1 1 Mbps, IDs 10-14, 20-24, 30-34
                                              Wire, QMI8658
```

Joint angles, rates, and targets are float32 degrees. Servo positions are saturated to the encoder range 0–4095. Non-finite targets are rejected. A finite angle outside ±180° is not rejected.

## Servo map

IDs follow Microduck. The bus always addresses all 15, in this order:

| IDs | Role |
| --- | --- |
| 10–14 | Leg |
| 20–24 | Leg |
| 30–33 | Neck and head. Policy order inside the 14-D action is 10–14, 30–33, 20–24. |
| 34 | Mouth. It is not part of the policy action. The web UI commands it from 0° to 30°. |

The default pose is maintained only in `python/rl_core.py`, in the servo order above:

```text
IDs 10–14:   0,  -5, -22, 0,  22 degrees
IDs 20–24:   0,   5,  22, 0, -22 degrees
IDs 30–34:  20,  20,   0, 0,   0 degrees
```

`action_scale` is 1. The policy home pose is derived from this one table. The STM32 follows the absolute angles sent by Linux; it has no default-pose table.

Every servo uses the fixed encoder center **2047** for angle conversion. There are no per-servo zero offsets, zero-position files, or APIs for changing that center. Calibration programs the servo's own center with its `mid` instruction, then holds raw position **2048**. That hold target does not change the fixed software center.

## First-time servo setup

**Scan all IDs** checks every address from **0 through 253** at the configured servo bus rate (1 Mbps), entering setup and unloading the servos automatically. **Stop scan** cancels the scan and keeps the IDs already found. Select a found ID to read or change that servo. There is no separate target-ID or Read-ID input. Selecting a result reads its current position and stored KP/KD; **Read** refreshes the readout without changing the Set KpKd inputs. Those inputs initially contain KP = 5 and KD = 10 and retain any edits when reading or selecting another servo. Addresses 254 (broadcast) and 255 are not assignable servo IDs.

To rename a servo, enter its current ID and the desired **Change ID to** value, then click **Change ID**. The app checks that the source responds and the destination is free, writes the ID, and verifies replies at the new and old addresses. A Bridge timeout aborts scanning; it is not reported as a missing servo. The normal Control page still monitors the robot's configured 15 joint IDs.

Deploy `assets/`, `python/`, and `sketch/` together for this feature; updating only the page or Python is insufficient. Restart the app through App Lab / App CLI so the updated sketch is compiled and uploaded.

Do this before the servos are installed in the frame. Connect **one servo at a time**. Two servos with the same ID on the bus will both accept a write.

1. Open **Servo setup** (`/servo.html`), scan, and select the connected servo from the results.
2. Assign IDs **10–14** and **20–24** for the legs. Assign **30–34** for the neck, head, and mouth. The ID write unlocks register `0x37` and then writes `0x05`.
3. Position the joint by hand, then press **Reset Mid**. Only the selected servo receives the `mid` instruction, **three times at 10 ms intervals**. It stays unloaded: no goal-position or torque-enable command follows.
4. Set stored KP/KD with **Set KpKd**. Both this write and **Change ID** first send **three writes of `0x00` to `0x37`**, then **three parameter writes**, with 10 ms between every command. KP/KD are stored together at `0x15`; the ID is stored at `0x05`. Changing an ID requires another scan before further editing.
5. Disconnect that servo and repeat until every ID exists exactly once.
6. Install the servos in the frame and finish the wiring: servo bus on D0/D1 (`Serial1`, 1 Mbps), QMI8658 on D20/D21 (`Wire`, 400 kHz, address `0x6A` then `0x6B`).

**All-servo protection** works without scanning or selecting an ID and has two independent rows. The first row's **Enable / Disable** buttons broadcast only the voltage/overcurrent protection switch; they do not read or write the current field. The second row's current input and **Set protection current** button broadcast only the current threshold, preserving the protection switch. The initial current field is 1.5 A; opening the page never writes either setting. Both commands use address `0xFE`, including connected servos outside the robot's configured 15 IDs. Overheat protection stays enabled and magnetic-encoder protection stays disabled when setting the switch, matching the reference SDK test. Existing voltage thresholds are unchanged. **Disable** clears the voltage and overcurrent conditions while keeping overheat protection.

The switch command writes only the unload-condition mask at `0x13`: enabled = `0x0D`, disabled = `0x04`. The separate current command writes only `0x1C` as a little-endian uint16 in 6.5 mA units (1.5 A rounds to 231, or 1.5015 A). Each command broadcasts `0x37 = 0` three times, then its single parameter three times, with 10 ms between commands. Servos remain unloaded. The reply confirms that the MCU sent that command; broadcasts have no individual servo acknowledgement or register readback.

After power-up the controller also writes the temporary torque register (address 40) and temporary KP/KD (addresses 50 and 51): KP = 6, KD = 20, and KP = 10 on ID 34. Torque off clears torque and does not write a goal position. Writing a goal after torque-off still drags a connected servo, so position writes are suppressed while disarmed.

## Wire formats

The browser uses HTTP/JSON on port 9527. Linux and STM32 use MessagePack RPC notifications over the 2 Mbps Bridge UART, carrying packed, little-endian binary payloads. `python/wire.py` and `sketch/protocol.h` define the same ABI. These sizes exclude the MessagePack envelope.

| Message | Direction | Payload bytes | Contents |
| --- | --- | ---: | --- |
| `duck_command` / `DQC1` | Linux → MCU, 50 Hz | 80 | magic, sequence/source time/TTL (3 × uint32), mode (uint8), 3 reserved bytes, 15 float32 target angles |
| `duck_state` / `DQS1` | MCU → Linux, 100 Hz | 240 | magic, version/flags (2 × uint8), servo mask (uint16), 10 uint32 timing/counter fields, raw accelerometer/gyro (6 × int16), angle/velocity/target (45 × float32) |
| `duck_servo` / `DQV2` | Linux → MCU, on demand | 12 | magic, op/ID/new ID/KP/KD (5 × uint8), flags/reserved (uint8), current/reserved (uint16) |
| `duck_servo_reply` / `DQS3` | MCU → Linux, on demand | 14 | magic, op/ID/ok/flags (4 × uint8), position or current (uint16), fixed center (uint16), KP/KD (2 × uint8) |
| `duck_cal` / `DQL2` | Linux → MCU | 10 | magic, sequence (uint32), operation/index (2 × uint8) |
| `duck_cal_reply` / `DQA1` | MCU → Linux | 10 | magic, sequence (uint32), index/ok (2 × uint8) |
| `duck_raw` / `DQR2` | MCU → Linux, 5 Hz normally, 100 Hz in setup/calibration | 38 | magic, online/held masks (2 × uint16), 15 uint16 positions |
| `duck_boot_pd` | MCU → Linux, about 1 Hz | 32 | online mask (uint16), 15 KP bytes, 15 KD bytes |

Protection switch operation **9** uses byte 5 = `0xFE`, bytes 6–8 = 0, byte 9 = condition mask, and bytes 10–11 = 0. Separate current operation **10** uses the same broadcast ID, bytes 6–9 = 0, and bytes 10–11 = current threshold. The 14-byte reply echoes only the switch mask at byte 7 for op 9, or only the threshold at bytes 8–9 for op 10; `ok` indicates MCU transmission only. Other operations still require bytes 9–11 of the request to be zero and cannot target `0xFE`.

The 80-byte command retains **3 reserved bytes** at offsets 17–19; both ends currently require them to be zero. State packets have no reserved bytes; flag bits 5–7 are unused, and servo-mask bit 15 is unused but rejected by the current decoder. The 12-byte servo packet uses its first reserved byte for protection op 9 and its final two reserved bytes for current op 10; unused reserved fields must remain zero. Extending any of these meanings requires updating both endpoints together. Packet sizes are checked exactly; arbitrary trailing bytes are not accepted.

STM32 ↔ servo uses Serial1 at 1 Mbps. Request frames are `FF FF ID LEN INSTRUCTION PARAMS CHECKSUM`; status frames replace INSTRUCTION with STATUS. `LEN = parameter/data bytes + 2`, and `CHECKSUM = ~(ID + LEN + instruction/status + parameters/data) & 0xFF`. Register words are little-endian. The bus uses ping `0x01`, read `0x02`, write `0x03`, mid `0x0B`, sync read `0x82`, and sync write `0x83`; broadcast ID is `0xFE`. Normal sync reads request 6 bytes from `0x38`; goals sync-write 6 bytes from `0x2A` per servo. This length-framed bus has no fixed reserved payload area: registers and instructions define any extension.

## Calibration

Do this after the harness is wired and every ID answers.

1. Open **Calibration** (`/calibrate.html`). Motion stops and all servos unload automatically. No position targets are sent on entry.
2. Turn one joint by hand to its mechanical center and press **Set center & hold** for that ID. The MCU sends the six-byte `FF FF ID 02 0B checksum` command **five times**, at least 10 ms apart. It waits **100 ms after the last command**, then enables only that servo with KP = 3, KD = 0 and writes position **2048**. Other joints are not moved or enabled.
3. Repeat for the remaining joints. A green row means the MCU has completed that joint's centering command sequence and enabled its center hold. No encoder position is captured or saved as an offset.
4. **Unload all** releases the centered joints for further manual adjustment. **Exit**, leaving the page, or a lost heartbeat unloads every servo.

Deploy the page, Python and sketch together. The calibration and raw-feedback packets use a new ABI; old packets that attempted to write software zero offsets are rejected.

## Running

Pinned stack: App CLI with Arduino Zephyr core 1.0.0, RouterBridge 0.4.3, RPClite 0.3.1, MsgPack 0.4.2. `sketch/sketch.yaml` lists those libraries, including the transitive ones. Python packages are installed by the App build from `python/requirements.txt`.

On the Uno Q, from this directory:

```bash
python3 tools/prepare-bridge.py
sudo bash tools/router-2m.sh
arduino-app-cli app start "$PWD"
```

`prepare-bridge.py` patches the resolved library cache and keeps the original headers as `.h.original`:

- RouterBridge 0.4.3: RPC thread stack 500 → 4096 bytes.
- RPClite 0.3.1: a bounded MessagePack scanner accepts a complete frame before `Unpacker::feed()`, and rejects a frame larger than the RPC buffer.

Re-run the script after a library-cache wipe or on a new board. If it cannot find the libraries, let the App CLI resolve `sketch.yaml` once, then run it again.

`router-2m.sh` sets the router serial rate to 2 Mbps in `/etc/systemd/system/arduino-router.service.d/90-xgoduck-baud.conf` and leaves the board GPIO ready-hook in place. That rate is global. Before starting another app that calls `Bridge.begin()` at 115200, move the drop-in aside, reload systemd, and restart `arduino-router`. Both ends must use the same rate. Restarting the router recreates its socket and resets the MCU, so stop this app first, restart the router, then start the app again. A normal app restart does not need a router restart.

If an isolated CLI build tries to re-download a toolchain that is already on the board and GitHub is unreachable, run `python3 tools/reuse-toolchain.py` and start the app again.

The `asm volatile("nop")` at the end of `setup()` keeps GCC 14 from emitting an out-of-range tail branch into firmware, which the Uno Q loader would reject with ENOEXEC (−8).

The web UI is served on port 9527. App Lab starts the same app under the name in `app.yaml`.

The app boots in **inference only**: the network runs, torque stays off. From the control page:

| Control | Effect |
| --- | --- |
| Inference only | Policy runs. Torque off. |
| Default pose | Holds the home pose. Requires a fresh MCU and IMU sample. |
| Walk / get up | Enables position control and the recovery state machine. |
| Pick | While upright in walk, runs `xgoduck_pick.onnx` for 4 s, then returns to walk. |
| Roulade | While upright in walk, runs `xgoduck_roulade.onnx` for 1.9 s, then returns to walk. |
| Torque off | Disables position control. |

The selected control mode remains active when the control page is idle, hidden or closed. Losing focus, hiding or leaving the page zeros travel commands without unloading. If the browser stops sending commands for 1 second, the host also zeros travel commands and keeps torque enabled. Use Torque off, Inference only or Calibration to unload. Calibration and servo setup retain their 2 second heartbeat and unload on exit.

Web walk commands are vx -0.2 to +0.4, vy ±0.3, ωz ±1.0, with a 15% dead zone on each joystick axis. Motion commands use the low-pass filter `v = 0.3 * old_v + 0.7 * new_v` on each outgoing command; Zero motion immediately clears the filter. Head channels are four radians in ±1. Keyboard: W/S, A/D, Q/E, Space.

## Policies

Models in `python/`:

- `xgoduck_walk.onnx`
- `xgoduck_getup.onnx`
- `xgoduck_pick.onnx`
- `xgoduck_roulade.onnx`

Each expects observation `[1, 61]` and emits action `[1, 14]`. All four are loaded and warmed at startup. Spare time in the 50 Hz loop runs one dummy pass on an idle session so a switch does not pay a cold start.

Observation layout: gyro (rad/s), gravity, 14 joint positions relative to home, 14 joint velocities, previous action, twist (3), head (4), and six zeros.

Recovery, evaluated on the 50 Hz clock:

- Tilt above 55° for 0.15 s → get-up model, action forced to 0 for 1 s, then the get-up action. Twist and head are zero.
- Tilt below 15° for 1 s → walk, and the web twist/head/mouth commands apply again.
- Pick, only from upright walk: command `[cos(2πφ), sin(2πφ), 0]` plus ten zeros, `φ` advances by `dt/4` (a 4 s cycle). The mouth opens to 30° and closes when φ ≥ 0.4.
- Roulade, only from upright walk: runs for 1.9 s with twist, head, and the six trailing command inputs at 0. Gyro, gravity, joint position, joint velocity, and the previous action stay on the usual path. Leg and neck smoothing drop to α = 0.15 for that window, then return to the previous values. Tilt does not select get-up until the 1.9 s ends.

Action smoothing defaults to leg α = 0.45 on slices `[0:5]` and `[10:14]`, and head α = 0.2 on slice `[5:10]`. `prev_action` stores the raw network output and is cleared on a policy switch. The 1 s fallen hold sends the home pose directly and does not keep a filtered walk action. Gyro smoothing α = 0.5, joint-velocity α = 0.4, gravity complementary-filter τ = 0.3 s. Accelerometer correction falls off when the norm leaves 1 g by more than about 20%. There is no gyro bias subtraction. The QMI axes are mapped once, in firmware; the host undoes the reference device mapping so the filter matches the Microduck math without applying that mapping twice.

Hardware-fault protection remains active: position control stops if host-to-MCU commands are absent for 250 ms or the IMU is absent for 100 ms. The host continues sending commands independently of browser activity. Enabling again after a fault takes an explicit mode change. Host feedback older than 150 ms also disarms; that matches the MCU source-age limit.

## Layout

```text
app.yaml                 App Lab manifest
assets/                  Web UI
python/                  Host policy, Bridge client, ONNX models
sketch/                  STM32 firmware
tools/                   Bridge preparation and router configuration
```

## Source-tree hygiene

Keep tests, debug scripts, captures, logs, model backups and build outputs outside the app source tree. Only runtime source, models, assets and installation tools belong in the distributed app. The App CLI manages its ignored `.cache` directory for the running Python environment; move completed sketch build outputs to an external development directory.

For a read-only runtime check, open `http://<board-ip>:9527/api/status`. It reports the fixed encoder center, default pose, live feedback and session-only centering state. Reading it does not enable motors.

## References

- [Microduck](https://github.com/pollen-robotics/microduck)
- [Arduino RouterBridge](https://github.com/arduino-libraries/Arduino_RouterBridge)
- [Arduino Router](https://github.com/arduino/arduino-router)
