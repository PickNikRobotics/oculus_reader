# oculus_reader

This is the active maintenance fork of the RAIL lab's oculus_reader. This version adds support for Quest 3 and updated OpenXR SDKs.

## ⚠️ Project Status: Quest 3 Support (Beta)

**Active Development:** I am the original creator of `oculus_reader`. I have launched this fork to provide support for the **Meta Quest 3** and modern OpenXR standards.

**Hardware Pledge:** Development for Quest 3 is currently ongoing. I am seeking a hardware pledge for integration testing to ensure a stable release.

**Support the Development:**
* **GitHub Sponsors:** [Click here to sponsor](https://github.com/sponsors/jborbik)
*   **PayPal (EUR):** [paypal.me/JBOrbik](https://paypal.me/JBOrbik)
*   **PayPal (USD):** [paypal.me/JedrzejOrbik](https://paypal.me/JedrzejOrbik)

Until the hardware is secured, please create issues if the software does not behave on-par with the original implementation from [rail-berkeley/oculus_reader](https://github.com/rail-berkeley/oculus_reader).

-------------------

## Quick start

System prerequisites (one-time):

```bash
sudo apt install git-lfs android-tools-adb
git lfs install
```

Clone (with the APK pulled correctly via Git LFS) and install into a local venv:

```bash
git clone git@github.com:PickNikRobotics/oculus_reader.git
cd oculus_reader
./install.sh
```

`install.sh` creates `.venv/` with `--system-site-packages` (so the venv can also see ROS 2 packages when ROS is sourced), installs the Python dependencies, installs `oculus_reader` in editable mode, smoke-tests the imports, and warns if `adb` is missing. Flags: `--venv <path>`, `--no-venv`, `--recreate`, `--help`.

### Activating the venv

```bash
source enter_venv.sh           # just the venv
source enter_venv.sh --ros     # also source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
```

`enter_venv.sh` must be sourced (it modifies the current shell). It locates the venv next to itself, so it works from any cwd. Exit with `deactivate`.

For ROS 2 use, **always source ROS before activating the venv** — the helper does this in the correct order when called with `--ros`.

### Quest-side setup

1. Enable Developer Mode for your Quest (via the Meta Quest mobile app: Settings → device → More Settings → Developer Mode). You'll need a Meta dev organization (free: <https://developer.oculus.com/manage/organizations/create/>).
2. Connect the Quest via USB-C and wear it.
3. Approve **Allow USB Debugging** + **Always allow from this computer** when prompted in the headset.
4. Verify: `adb devices` should list your headset.

### Running

Stream controller poses to the terminal:

```bash
source enter_venv.sh
python oculus_reader/reader.py
```

Publish controller poses as TF frames (`world` → `oculus_r` / `oculus_l`) for visualization in RViz2:

```bash
source enter_venv.sh --ros
python oculus_reader/visualize_oculus_transforms_ros2.py
```

Then in another ROS-sourced terminal (same `ROS_DOMAIN_ID` and `ROS_DISTRO`):

```bash
rviz2     # add a TF display, fixed frame = world
```

**Note**: the Quest's proximity sensor suspends the teleop APK when nobody is wearing the headset. If `ros2 topic echo /tf` shows nothing, first check that the publisher terminal is logging non-empty `buttons: {...}` — if it's silent, put the headset on (or defeat the prox sensor).

## Teleoperation

`oculus_reader/teleoperate.py` plus `launch/teleoperate.launch.py` provide a full clutched-VR-teleop pipeline: the Quest controllers are mapped onto two TF reference frames; while the grip is held, the node also publishes a Cartesian velocity command that drives the robot's end effector toward the reference.

Because the velocity command uses the custom `moveit_pro_controllers_msgs/msg/VelocityForceCommand` type, **this part of the pipeline is intended to run inside the MoveIt Pro dev container** (where that message package is built and on PYTHONPATH). Quest connectivity (ADB) is also needed; if your container doesn't have it, install `pure-python-adb` (`pip3 install --user pure-python-adb`) and make sure `/dev/bus/usb` is passed through.

### Build (inside the dev container)

`oculus_reader` is an `ament_python` package (see `package.xml`, `setup.cfg`, `setup.py`, `resource/oculus_reader`). Build with colcon:

```bash
cd /home/studio-user/user_ws        # path inside the dev container
colcon build --packages-select oculus_reader
source install/setup.bash
```

After that, `ros2 launch` and `ros2 run` both work:

```bash
ros2 launch oculus_reader teleoperate.launch.py
# or, directly:
ros2 run oculus_reader teleoperate
```

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
- The tip frame name is hard-coded as `TIP_FRAME_ID = 'grasp_link'` at the top of `teleoperate.py` — change there if your robot uses a different convention.
- The `velocity_force_controller` must be running on the robot (the topic `/velocity_force_controller/command` should appear in `ros2 topic list`). Otherwise commands are published but go nowhere.

### Launch

```bash
ros2 launch oculus_reader teleoperate.launch.py
```

This brings up:
1. A `static_transform_publisher` for `world → quest_origin`, defining where the Quest tracking frame sits relative to the robot.
2. The `teleoperate.py` node, publishing the four TF frames listed above and the velocity command.

### Launch arguments

| Arg | Default | Meaning |
|---|---|---|
| `qx`, `qy`, `qz`, `qw` | `0.5, 0.5, 0.5, 0.5` (xyzw) | Quest→world rotation. The default assumes operator stands behind/beside the robot, facing in the same direction as the robot. See "Calibration" below. |
| `world_frame` | `world` | Robot world frame (parent of `quest_origin`). |
| `quest_frame` | `quest_origin` | TF parent of the published controller/reference frames. |
| `linear_gain` | `1.0` | m/s of EE velocity per metre of position error. |
| `angular_gain` | `1.0` | rad/s of EE angular velocity per rad of orientation error. |

Examples:

```bash
# Tune the gains:
ros2 launch oculus_reader teleoperate.launch.py linear_gain:=1.5 angular_gain:=0.8

# Override the orientation (if you stand facing the robot):
ros2 launch oculus_reader teleoperate.launch.py qx:=... qy:=... qz:=... qw:=...
```

You can also set the same parameters via `ros2 run`:

```bash
ros2 run oculus_reader teleoperate --ros-args -p linear_gain:=1.5 -p angular_gain:=0.8
```

Other parameters not currently exposed on the launch file (you can pass via `ros2 run --ros-args -p ...`):

| Param | Default | Meaning |
|---|---|---|
| `parent_frame_id` | `world` (launch overrides to `quest_origin`) | TF parent of published frames. |
| `cmd_topic` | `/velocity_force_controller/command` | Where to publish the velocity command. |
| `drive_hand` | `'r'` | Which controller drives the robot. Set to `'l'` to drive from the left hand. |
| `gripper_action_name` | `/robotiq_gripper_controller/gripper_cmd` | Where to send `GripperCommand` goals. |
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

-------------------

# Original README from [rail-berkeley/oculus_reader](https://github.com/rail-berkeley/oculus_reader):
This repository provides the tool to read the position and pressed button from the Oculus Quest device.

Oculus reader consits of two elements: python script which receives the readings from the APK and the APK itself. Currently the pose of the controllers and pressed buttons are transfered from the APK. This behavior can be extended using provided APK [source code](app).

## Clone the repository

To pull the APK correctly, Git LFS has to be configured before cloning the repository. The installation is described here https://git-lfs.github.com. On Ubuntu follow these steps:

```bash
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs
git lfs install # has to be run only once on a single user account
```

Now you can clone this repository either with HTTPS or SSH.

If you decide to use the HTTPS protocol, you can reduce the number of authentification prompts when pushing/pulling caused by Git LFS with:

```bash
git config lfs.https://github.com/jborbik/oculus_reader.git/info/lfs.locksverify false
```

This command has to be run from the repository workspace.

## Setup of the ADB

[ADB](https://developer.android.com/studio/command-line/adb) is required for the communication between Oculus Quest and the python reader script.

To install ADB on Ubuntu run:

```
sudo apt install android-tools-adb
```

On other systems follow the steps from the 'app' folder.

### Set up of a new Oculus Quest device

[!WARNING]
This section needs to be updated.

1. Determine your Oculus Quest account name:
If you haven’t used Oculus Quest before, start it and follow the steps to create your profile and get yourself started. Otherwise follow these steps to find out your username:
    1. Go to: [https://www.oculus.com/](https://www.oculus.com/) 
    2. Log in to account:
    ![image_0](https://user-images.githubusercontent.com/14967831/106832581-c7288f00-6646-11eb-91e0-3b74e81a58ba.png)
    3. After logging in **select your profile again** in top right corner and select **‘Profile’**
    ![image_1](https://user-images.githubusercontent.com/14967831/106832585-c859bc00-6646-11eb-9a3d-3a55f844ee37.png)
    4. You will be able to see your username on the following screen:
    ![image_2](https://user-images.githubusercontent.com/14967831/106832678-f7702d80-6646-11eb-823e-1001d6bffe01.png)
2. Enable Oculus Quest development mode:
    1. If you belong to RAIL, inform me (Jedrzej Orbik) that you need to join the development organization. This is required to activate debugging mode on the device. Otherwise create your own organization <https://developer.oculus.com/manage/organizations/create/> and fill in the appropriate informaiton.
    2. Turn on the device you want to use for development.
    3. Open the Oculus app on your phone and then go to **Settings**.
    4. Tap the device and then go to **More Settings** > **Developer Mode**.
    5. Turn on the **Developer Mode** toggle.
    6. Connect your device to your computer using a USB-C cable and then wear the device.
    7. Accept **Allow USB Debugging** and **Always allow from this computer** when prompted to on the device.  
        ![image_3](https://user-images.githubusercontent.com/14967831/104061507-048d2e80-51f9-11eb-8327-7917f6a1ab60.png)  
    8. (Windows only) Install the Oculus ADB Drivers
        1. Download [the zip file containing the driver](https://developer.oculus.com/downloads/package/oculus-adb-drivers/).
        2. Unzip the file.
        3. Right-click on the .inf file and select **Install**.

## How to run the code

If you intend to use the precompiled APK with the predefined behavior, where the position and the pressed buttons are transferred, please follow the steps from the [scripts folder](oculus_reader/README.md). If you plan to extend the app, please read the README from the [app folder](app/README.md).

### Communication using the USB cable (easier to set up)

1. Connect Oculus Quest to PC with USB cable. This is required to establish the connection.
2. Run the exemplary file: `python oculus_reader/reader.py`
3. (optionally) If ROS is installed on your machine, the current transformation can be visualized using the script [visualize_oculus_transforms.py](oculus_reader/visualize_oculus_transforms.py).

### Communication over the network (more portable)

1. Make sure that Oculus Quest is connected to the same network as the computer.
2. Connect Oculus Quest to PC with USB cable. This is required to establish the connection.
3. Put on the headset and allow the permission as requested.
4. Verify that a device is visible with: `adb devices`. The expected output:  
`List of devices attached`  
`    ce0551e7                device`
5. Check the IP address of the headset:  
    `adb shell ip route`  
    Expected output:  
    `10.0.30.0/19 dev wlan0  proto kernel  scope link  **src **10.0.32.101`
6. Read the IP address of the device standing after `**src`.
7. Provide the IP address when creating OculusReader object.
8. Run the exemplary file: `python oculus_reader/reader.py`
9. (optionally) If ROS is installed on your machine, the current transformation can be visualized using the script [visualize_oculus_transforms.py](oculus_reader/visualize_oculus_transforms.py).

### Stopping the app from adb
It is possible to stop the app from adb. Use: `adb shell am force-stop com.rail.oculus.teleop`

## Citation

If you find this work helpful, please consider citing it using the following reference:
```
@misc{OrbikEbert2021OculusReader,
  author = {Jedrzej Orbik, Frederik Ebert},
  title = {Oculus Reader: Robotic Teleoperation Interface},
  year = {2021},
  url = {https://github.com/rail-berkeley/oculus_reader},
  note = {Accessed: YYYY-MM-DD}
}
```
