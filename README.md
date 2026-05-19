# oculus_reader (PickNik fork)

A PickNik fork of the original [rail-berkeley/oculus_reader](https://github.com/rail-berkeley/oculus_reader)
(via [jborbik/oculus_reader](https://github.com/jborbik/oculus_reader) for Quest 3 support). This README
covers only the PickNik-specific additions on top of the upstream library:

- The `data_collection` ROS 2 package that turns the Quest controller stream into TF frames, a Cartesian
  velocity command, and a gripper command for VR teleop in MoveIt Pro.
- `container_install.sh` for one-shot dev-container setup.

For the underlying OculusReader library itself -- how to install ADB, enable Quest developer mode, set up
USB debugging, the APK communication model, etc. -- see the upstream README:
**<https://github.com/rail-berkeley/oculus_reader>**

-------------------

## Repo layout

This repository contains two pieces:

| Directory | What it is |
|---|---|
| `oculus_reader/` | The upstream Python module (`reader.py`, button parser, APK). Pure Python, no ROS. Provides the `OculusReader` class used to talk to the Quest. |
| `data_collection/` | A ROS 2 `ament_python` package (the Quest -> TF / Cartesian velocity / gripper teleop pipeline). Imports `oculus_reader` at runtime. |

The `oculus_reader/` module is not pip-installed (a setup.py at the repo root would shadow `data_collection/` from colcon's package discovery); instead `container_install.sh` drops a small `.pth` file so `import oculus_reader` resolves to the source tree directly.

## Quick start

This package is intended to run inside the MoveIt Pro dev container, where ROS 2, `moveit_pro_controllers_msgs`, and the rest of the MoveIt Pro stack are already available. The container is the only supported environment.

### Clone

Clone into your `src/` workspace directory on the host. Git LFS is required to pull the APK:

```bash
sudo apt install git-lfs           # if you don't already have it
git lfs install                    # once per user account
cd ~/picknik/workspaces/moveit_pro_example_ws/src
git clone git@github.com:PickNikRobotics/oculus_reader.git
```

### One-time container setup

Inside the container, the teleop needs `adb`, a few small pip packages (`pure-python-adb`, `numpy`, `pyyaml`), and a `.pth` entry so `import oculus_reader` finds the module in this repo. The container is the isolation boundary, so everything goes system-wide -- no venv. Run once per container build:

```bash
cd /home/studio-user/user_ws/src/oculus_reader
bash container_install.sh
```

The script auto-elevates with `sudo` if needed, is idempotent (safe to re-run), and verifies the imports at the end. If you maintain the dev container image, you can equivalently fold its contents into the Dockerfile so the deps are baked in.

### Build

`data_collection` is an `ament_python` package (see `data_collection/package.xml`, `data_collection/setup.cfg`, `data_collection/setup.py`, `data_collection/resource/data_collection`). Build with colcon:

```bash
cd /home/studio-user/user_ws
colcon build --packages-select data_collection
source install/setup.bash
```

After that, `ros2 launch` and `ros2 run` work in any terminal that sources the overlay:

```bash
ros2 launch data_collection data_collection.launch.py
# or, directly:
ros2 run data_collection data_collection
```

### Quest-side setup

Enable developer mode on the Quest, approve USB debugging, and verify connectivity with `adb devices`. See the [upstream README](https://github.com/rail-berkeley/oculus_reader#set-up-of-a-new-oculus-quest-device) for the step-by-step.

### Smoke-test tools (optional)

Two small standalone scripts are useful for debugging the controller link without running the full teleop pipeline. They live in the `oculus_reader/` Python module:

```bash
# Stream raw controller poses + buttons to the terminal.
python3 /home/studio-user/user_ws/src/oculus_reader/oculus_reader/reader.py

# Publish controller poses as TF frames (world -> oculus_r / oculus_l) for RViz2.
python3 /home/studio-user/user_ws/src/oculus_reader/oculus_reader/visualize_oculus_transforms_ros2.py
```

In another terminal:

```bash
rviz2     # add a TF display, fixed frame = world
```

**Note**: the Quest's proximity sensor suspends the teleop APK when nobody is wearing the headset. If `ros2 topic echo /tf` shows nothing, first check that the publisher terminal is logging non-empty `buttons: {...}` -- if it's silent, put the headset on (or defeat the prox sensor).

## Teleoperation

`data_collection/data_collection/data_collection.py` plus `data_collection/launch/data_collection.launch.py` provide a full clutched-VR-teleop pipeline: the Quest controllers are mapped onto two TF reference frames; while the grip is held, the node also publishes a Cartesian velocity command that drives the robot's end effector toward the reference, and the analog trigger commands the gripper.

### What gets published

Two pairs of TF frames, under a `quest_origin` parent that is itself a child of the robot's `world` frame:

| Frame | Description | When it updates |
|---|---|---|
| `oculus_r`, `oculus_l` | Raw controller poses, every tick. | Continuously, ~20 Hz. |
| `oculus_r_reference`, `oculus_l_reference` | The pose the robot's end effector should track. | Only while the corresponding grip button is held. |

Plus the velocity command and the gripper command:

| Topic / Action | Type | When it publishes |
|---|---|---|
| `/velocity_force_controller/command` | `moveit_pro_controllers_msgs/msg/VelocityForceCommand` | Every tick while the configured drive hand's clutch is engaged, plus one zero-twist on the falling edge so the robot stops promptly. |
| `/robotiq_gripper_controller/gripper_cmd` (action) | `control_msgs/action/GripperCommand` | A new goal is sent whenever the analog trigger maps to a joint position more than `gripper_threshold` rad away from the last sent value. |

The reference frames are **clutched**: they only move when the grip button on that hand's controller is held down. When you release the grip, they freeze. When you press the grip again, the reference snaps to the **current** robot tip pose (looked up via TF) — so the robot never jumps even if it has moved between presses. Motion deltas are applied in the **parent (world) frame**, so "move your hand down in world" always means "reference moves down in world", regardless of the robot's current orientation.

### Prerequisites

- The robot's TF tree must publish `grasp_link` (the tip frame the references anchor to). This typically means a `robot_state_publisher` is running with the robot's URDF. Without it, the teleop node will log "Waiting for TF '<parent>' -> 'grasp_link'..." and never start.
- The tip frame name is hard-coded as `TIP_FRAME_ID = 'grasp_link'` at the top of `data_collection/data_collection/data_collection.py` — change there if your robot uses a different convention.
- The `velocity_force_controller` must be running on the robot (the topic `/velocity_force_controller/command` should appear in `ros2 topic list`). Otherwise commands are published but go nowhere.

### Launch

```bash
ros2 launch data_collection data_collection.launch.py
```

This brings up:
1. A `static_transform_publisher` for `world → quest_origin`, defining where the Quest tracking frame sits relative to the robot.
2. The teleop node, publishing the four TF frames listed above and the velocity command.

### Launch arguments

| Arg | Default | Meaning |
|---|---|---|
| `qx`, `qy`, `qz`, `qw` | `0.5, 0.5, 0.5, 0.5` (xyzw) | Quest→world rotation. The default assumes operator stands behind/beside the robot, facing in the same direction as the robot. See "Calibration" below. |
| `world_frame` | `world` | Robot world frame (parent of `quest_origin`). |
| `quest_frame` | `quest_origin` | TF parent of the published controller/reference frames. |
| `linear_gain` | `1.0` | m/s of EE velocity per metre of position error. |
| `angular_gain` | `1.0` | rad/s of EE angular velocity per rad of orientation error. |
| `cmd_topic` | `/velocity_force_controller/command` | Topic for the `VelocityForceCommand` sent to the arm controller. |
| `gripper_action_name` | `/robotiq_gripper_controller/gripper_cmd` | `GripperCommand` action name. |

Examples:

```bash
# Tune the gains:
ros2 launch data_collection data_collection.launch.py linear_gain:=1.5 angular_gain:=0.8

# Override the orientation (if you stand facing the robot):
ros2 launch data_collection data_collection.launch.py qx:=... qy:=... qz:=... qw:=...
```

You can also set the same parameters via `ros2 run`:

```bash
ros2 run data_collection data_collection --ros-args -p linear_gain:=1.5 -p angular_gain:=0.8
```

Other parameters not currently exposed on the launch file (you can pass via `ros2 run --ros-args -p ...`):

| Param | Default | Meaning |
|---|---|---|
| `parent_frame_id` | `world` (launch overrides to `quest_origin`) | TF parent of published frames. |
| `drive_hand` | `'r'` | Which controller drives the robot. Set to `'l'` to drive from the left hand. |
| `gripper_drive_hand` | `'r'` | Which controller's trigger commands the gripper. Independent of `drive_hand`. |
| `gripper_min_position` | `0.0` (rad) | Knuckle joint position when the trigger is fully released. |
| `gripper_max_position` | `0.8` (rad) | Knuckle joint position when the trigger is fully pressed. |
| `gripper_threshold` | `0.02` (rad) | Dead-band: a new goal is sent only when the target moves by more than this since the last sent goal. Prevents action-server churn. |
| `gripper_max_effort` | `50.0` (N) | `GripperCommand.command.max_effort`. Lower to make the gripper stall earlier on contact. |

### Tuning the gains

Gain `N` makes the controller try to close `N×` of the current error per second, i.e. a time constant of `1/N` seconds. Start at `1.0` (1 s) and adjust:

- **Oscillates / overshoots at engagement** → drop the gain.
- **Robot lags behind your hand** → raise the gain.
- **Asymmetric is normal**: linear gain can often go higher than angular gain because joint-velocity limits on rotations are stricter for most arms.

A reasonable starting envelope is `linear_gain ∈ [0.5, 2.0]`, `angular_gain ∈ [0.3, 1.5]`.

### Calibration

The static TF rotation in the launch file should match the **relative orientation** between the human operator and the robot at recenter time. Translation is fixed at zero — only the orientation matters for the clutch math.

The default orientation assumes the operator is **behind (or to the side of) the robot, facing in the same direction as the robot** — i.e., the operator's "forward" is aligned with the robot's "forward". This is the most common stance for desktop teleop. If you instead want to **face the robot** (the operator's "forward" pointing toward the robot, opposite to the robot's "forward"), you'll need to override the quaternion or motions will feel inverted in X and Y.

Calibration recipe:
1. Stand on a fixed, marked spot near the robot.
2. Stand still for a couple of seconds, then long-press the Quest's Oculus button to **recenter** tracking.
3. Run the launch file.
4. In RViz2 (fixed frame = `world`, TF display), wave the right controller and verify:
   - "Hand up" → `oculus_r` moves up.
   - "Hand forward" (away from your body) → `oculus_r` moves toward the robot.
   - "Hand right" → `oculus_r` moves to your right.
5. Hold the right **grip** and move the controller — `oculus_r_reference` should follow with no jump, and the robot should track it.

### Button conventions

Per hand, OculusReader exposes:

| Button | Used for |
|---|---|
| Grip (`RG` / `LG`) | **Clutch** — hold to drive the reference frame (and, on the `drive_hand`, the robot). |
| Trigger analog (`rightTrig` / `leftTrig`, float in [0, 1]) | **Gripper** — linearly mapped to `[gripper_min_position, gripper_max_position]`. Fully released → open, fully pressed → closed. |
| A / B / X / Y, joysticks | Unused so far. |

The gripper is driven continuously, not binary — recording the analog value gives a richer signal for downstream policy learning, and you can always threshold it later if a binary signal is all you need.

### Logs you'll see

```
[INFO] [oculus_teleop]: Publishing under 'quest_origin'. Waiting for TF 'quest_origin' -> 'grasp_link'...
[INFO] [oculus_teleop]: References initialized at 'grasp_link' pose. Waiting for Quest data...
[INFO] [oculus_teleop]: Streaming started. Controllers detected: right, left. Hold the grip button to engage the clutch.
[INFO] [oculus_teleop]: oculus_r_reference: clutch engaged
[INFO] [oculus_teleop]: oculus_r_reference: clutch released
```

No per-tick spam. Clutch state changes log on each transition.

## Citation

The underlying OculusReader library is from:

```
@misc{OrbikEbert2021OculusReader,
  author = {Jedrzej Orbik, Frederik Ebert},
  title = {Oculus Reader: Robotic Teleoperation Interface},
  year = {2021},
  url = {https://github.com/rail-berkeley/oculus_reader},
}
```
