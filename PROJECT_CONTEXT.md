# Curious Robot – Project Context

## 1. Project Vision

The goal of this project is to build a small autonomous robot with childlike curiosity.

The robot should not simply execute predefined commands. Long-term, it should:

- explore its environment autonomously
- detect unfamiliar objects and situations
- distinguish known from unknown things
- ask a human questions such as "What is this?"
- remember the answer persistently
- recognize previously learned objects later
- gradually build knowledge about its environment
- develop simple internal drives such as curiosity, boredom, fear, energy and social need
- use its experiences to influence future behavior

The inspiration is conceptually similar to a curious fictional companion robot, but the robot should have its own design and architecture.


## 2. Long-Term Architecture

The planned high-level architecture is:

Perception
    ↓
Novelty / Uncertainty Detection
    ↓
Memory / World Model
    ↓
Internal Drives / Personality
    ↓
Behavior Decision
    ↓
Navigation / Action

Possible future hardware:

- Raspberry Pi
- ESP32
- differential drive
- wheel encoders
- camera
- LiDAR or ToF sensors
- IMU
- microphone
- speaker
- servos for head movement

The project is currently simulation-first.


## 3. Software Stack

Current:

- macOS
- Apple Silicon Mac
- VS Code
- Git / GitHub
- Gazebo Harmonic
- SDF
- Python for LiDAR perception, reactive navigation and world-pose/path tracking
- Matplotlib for offline trajectory plots
- working Python bindings for `gz.transport13` and `gz.msgs10` (confirmed by the owner)

Planned:

- ROS 2 if appropriate
- computer vision
- pretrained vision embeddings
- persistent memory
- active learning
- reinforcement learning later


## 4. Important macOS Gazebo Detail

On this macOS setup, Gazebo server and GUI cannot be started together using a normal:

    gz sim world.sdf

Instead, they are started separately.

Terminal 1:

    gz sim -s simulation/worlds/basic_world.sdf

Terminal 2:

    gz sim -g

Custom robot models are located under:

    simulation/robot/

Before starting Gazebo, the resource path is currently set using:

    export GZ_SIM_RESOURCE_PATH="$PWD/simulation/robot:$GZ_SIM_RESOURCE_PATH"


## 5. Current Git Workflow

Current development branch:

    feature/session-03-localization

Development should happen incrementally on feature branches.

Do not make large unrelated changes at once.


## 6. Current Project Structure

Current relevant structure:

    curious-robot/
    ├── README.md
    ├── .gitignore
    ├── AGENTS.md
    ├── PROJECT_CONTEXT.md
    │
    ├── simulation/
    │   ├── worlds/
    │   │   └── basic_world.sdf
    │   │
    │   └── robot/
    │       └── curious_robot/
    │           ├── model.config
    │           └── model.sdf
    │
    ├── src/
    │   ├── localization/
    │   │   ├── __init__.py
    │   │   ├── pose.py
    │   │   ├── path.py
    │   │   ├── record.py
    │   │   └── plot.py
    │   ├── navigation/
    │   │   ├── navigation.py
    │   │   └── movement.py
    │   ├── perception/
    │   │   ├── lidar.py
    │   │   └── test.py
    │   ├── memory/
    │   └── brain/
    │
    ├── config/
    └── tests/
        ├── test_localization.py
        ├── test_localization_plot.py
        ├── test_navigation.py
        └── test_perception.py

`src/perception/test.py` displays live LiDAR data. Navigation and movement are
implemented in `src/navigation/`; localization records the simulator's model pose
independently in `src/localization/`. `memory`, `brain` and `config` remain empty.
Tests use `unittest`; the two plot tests use Matplotlib and skip if it is absent.
Local `.venv/` and generated `recordings/` CSV/images are ignored by Git.


## 7. Current Simulation World

File:

    simulation/worlds/basic_world.sdf

The world currently contains:

- physics configuration
- explicit Physics, UserCommands, SceneBroadcaster and Sensors systems (Ogre2)
- lighting
- 10 m x 10 m floor
- four surrounding walls
- three static box obstacles
- the Curious Robot model

The three obstacles have different sizes and colors.

The world successfully loads in Gazebo.


## 8. Current Robot

Robot model:

    simulation/robot/curious_robot/model.sdf

Model metadata:

    simulation/robot/curious_robot/model.config

The robot currently contains:

- chassis
- left wheel
- right wheel
- revolute joint for left wheel
- revolute joint for right wheel
- passive spherical support (`caster_support`) with a ball joint
- Gazebo Harmonic `gz::sim::systems::DiffDrive` system
- chassis-mounted planar `gpu_lidar` sensor and a small visual housing

The stable mechanical layout uses wheel centers at `(-0.10, +/-0.23, -0.10)` m,
wheel radius `0.10` m, and wheel separation `0.46` m. Both wheel joint axes are
`0 1 0`, explicitly expressed in the model frame. The support center is at
`(0.18, 0, -0.16)` m with radius `0.04` m.

The drive system uses the existing `left_wheel_joint` and `right_wheel_joint`.
It accepts `gz.msgs.Twist` on `/model/curious_robot/cmd_vel`:
`linear.x` is forward speed in m/s, and `angular.z` is left-turn speed in rad/s.
ROS 2 is not required.

The LiDAR is a sensor within the chassis link, so it follows the chassis without
another joint or changes to the existing masses/collisions. Its housing is visual
only. The chassis top is at local z = 0.20/2 = 0.10 m. The scan origin is
`(0.125, 0, 0.15)` m: x = 0.50/4 and z = 0.10 + 0.05. At the initial model
height of 0.25 m, the scan plane is at world z = 0.40 m, above the chassis and
housing and within the vertical extent of the walls and all three obstacles.

LiDAR configuration:

- topic: `/model/curious_robot/lidar`
- message: `gz.msgs.LaserScan` (Gazebo Transport, no ROS 2)
- 361 horizontal samples from -pi to +pi, including both endpoints
- 1 degree angular spacing; the first and last rays share the rear direction
- one horizontal scan plane, 0.1–10 m range, 10 Hz in simulation time
- Ogre2 rendering via the world's Sensors system

Explicit world plugins replace Harmonic's default server plugins, so the existing
Physics, UserCommands and SceneBroadcaster systems are also declared in the world.
The arena geometry, robot mechanics and differential-drive configuration are unchanged.

The robot is dynamic:

    <static>false</static>

The robot successfully appears in Gazebo.

Its spawn height has already been adjusted so that it is no longer embedded in the floor.

The world currently controls the robot spawn pose rather than defining the global position inside the robot model.


## 9. Current Robot Concept

The robot uses differential drive.

Conceptually:

    left wheel  +  right wheel
          \        /
           \      /
            chassis

Both wheels rotating forward:
    robot moves forward

Different wheel velocities:
    robot turns

Opposite wheel velocities:
    robot rotates approximately in place.


## 10. Current Development Session

    Session 3 – Localization & Path Tracking

Sessions 1 (simulation/perception) and 2 (reactive navigation) are complete.
The owner confirms that autonomous driving works stably in Gazebo.
Session 3 adds simulator ground-truth x/y/yaw, sampled CSV recording and an
offline Matplotlib trajectory plot. It does not estimate pose from LiDAR or
wheel odometry. Localization and navigation run as separate processes.

Stop after Session 3 for owner review. Do not add mapping, SLAM, destination
navigation, path planning, Brain, Memory, AI or ROS 2. Do not merge into main.


## 11. Session 3 Definition of Done

Implemented and verified:

1. Receive the existing Gazebo model world pose from Python.
2. Extract x/y and quaternion-derived yaw with tested conventions.
3. Record sampled `(timestamp, x, y, yaw)` independently of navigation.
4. Plot the trajectory with start/end points and final heading.
5. Review navigation and make only a targeted safety fix.
6. Run all perception, navigation, localization and plot tests successfully.

Live checks and remaining manual checks are described in section 13.


## 12. Completed

Already working:

- Gazebo Harmonic installed
- Gazebo GUI works
- custom SDF world works
- floor works
- four walls work
- lighting/materials work
- three obstacles work
- custom robot model loads
- chassis exists
- two wheels exist
- wheel joints exist
- robot appears at the correct height
- passive support and wheel placement provide stable balance (manually confirmed)
- differential drive working (forward/backward, turns, rotation and stop manually confirmed)
- LiDAR working: raw ranges verified for walls and all three obstacles, with changes
  during driving, rotation, and a controlled near/far obstacle test

Session 1 and Session 2 are complete, including eight-sector perception,
invalid-measurement handling and reactive navigation. Stable autonomous driving
is confirmed by the owner. The 1.0-m threshold is now reflected in all tests and
code descriptions. Session 3 adds working world-pose reception, sampling, CSV
recording and plotting. Brain and Memory have not been started; ROS 2 is not used.


## 13. Current Implementation and Validation

### Current perception state

The code is split into two small modules:

- `src/perception/lidar.py`: sector evaluation, scan validation and table formatting;
  no Gazebo imports or side effects.
- `src/perception/test.py`: live subscription to `/model/curious_robot/lidar`,
  callback and `main()` entry point. Gazebo bindings are imported only at startup.
  The callback passes sensor range limits and horizontal angle metadata to the
  evaluator. Invalid scans produce an explicit unknown-status message.

The owner changed `OBSTACLE_DISTANCE` in `src/perception/lidar.py` from 2.5 to
1.0 m. This is the shared threshold used by the perception status for every sector
and by navigation for the front sector. The existing sector boundaries are unchanged:

| Sector | Python slice(s) |
|---|---|
| Front | `[158:203]` |
| Front-left | `[203:248]` |
| Left | `[248:293]` |
| Back-left | `[293:338]` |
| Back | `[338:361] + [0:23]` |
| Back-right | `[23:68]` |
| Right | `[68:113]` |
| Front-right | `[113:158]` |

For this scan, index 180 points forward, 225 front-left, 270 left, 315 back-left,
0/360 backward, 45 back-right, 90 right, and 135 front-right. Indices 0 and 360
represent the same direction and remain included in the rear sector.

Measurement handling:

- Finite values within the message's `range_min` / `range_max` are accepted.
- Positive infinity means no return within range, not a measured finite distance.
- NaN, negative infinity, zero, negative values, out-of-range values and nonnumeric
  entries are excluded and counted per sector.
- A valid distance below 1.0 m produces `JA`, even with other invalid values.
- Without a detected near obstacle, invalid/missing sector values produce
  `UNBEKANNT` rather than a false clear indication.
- Fully valid sectors without a near obstacle produce `NEIN`. At exactly 1.0 m,
  the strict threshold does not classify the return as a near obstacle.
- Missing distances display `--`; all-no-return sectors display `kein Treffer`.
- Scans must have 361 samples and the expected -pi start / one-degree step.
  Invalid range limits or incompatible layouts are rejected, without reusing
  earlier results. The next valid scan can still be processed.

The table shows all eight sectors with minimum distance, obstacle status and
invalid-value count. `NEIN` applies only to the current sector measurements and
threshold; it is not a navigation or path-clearance decision. No stale-data
watchdog is implemented: an old terminal table is not proof of a current scan.

Run the live display from the repository root:

```bash
python3 src/perception/test.py
```

Start the Gazebo server and GUI separately as described in section 4 and press
Run before expecting live scans. Use a third terminal with the Python environment
that has the Gazebo bindings installed.

Live data flow:

1. Gazebo publishes a `gz.msgs.LaserScan` on `/model/curious_robot/lidar` at
   approximately 10 Hz in simulation time.
2. The subscription invokes `lidar_callback(msg)` whenever a message arrives.
3. `analyze_scan()` checks the scan and calls `evaluate_sector()` for each sector.
4. `format_scan()` builds the table; the callback prints it to the terminal.

Each received scan appends a new table; the display does not overwrite the
previous table or clear the terminal. The `while True` loop with `time.sleep(1)`
only keeps the subscriber alive: it does not poll the sensor or set the scan rate.
Ctrl+C ends the Python display while Gazebo continues running. Pausing Gazebo or
losing the scan stream leaves the last table visible without a stale-data warning.
The display only receives data and prints results; it sends no movement commands.

### Automated perception tests

`tests/test_perception.py` uses Python's built-in `unittest`. It generates synthetic
measurements, calls the perception functions and compares their results with
expected values. It does not subscribe to a real sensor, launch Gazebo or move
the robot. Importing the live-display module for the callback tests does not run
`main()` or import the Gazebo bindings.

For example, sector values `[4.0, NaN]` must produce a distance of 4.0 m,
an unknown obstacle status and one invalid measurement. This checks that a
missing measurement cannot silently become a clear-sector decision.

Run the tests from the repository root without Gazebo or additional dependencies:

```bash
python3 -m unittest discover -s tests -v
```

Ten perception tests cover invalid values, missing data, infinity, sensor and
obstacle boundaries, sector edges including the rear wrap, scan metadata,
formatting and callback recovery. Five outdated assertions were corrected for
the already-existing 1.0-m threshold; the perception algorithm was not changed.
The old 2.5-m sensor-limit error message now displays the actual constant.

### Current navigation

The owner introduced `src/navigation/navigation.py` and `movement.py`.
The original decision logic moved forward while turning toward a blocked front,
reset its turn direction immediately when the front cleared, and then accelerated
to 0.4 m/s. It did not reliably check invalid diagonal measurements, catch scan
validation failures or stop on missing scans.

The corrected `choose_motion()` function is independent of Gazebo and returns
linear speed, angular speed and the retained turn direction:

- Normal forward speed is 0.15 m/s.
- A front distance strictly below 1.0 m starts avoidance.
- A diagonal distance below 0.6 m also blocks forward motion.
- The initially more open front diagonal determines the turn direction; ties go
  right. That direction is retained throughout the maneuver.
- Avoidance rotates in place at +/-0.5 rad/s, with zero forward speed.
- Forward motion resumes only at front distance >= 1.3 m and both front diagonals
  >= 0.8 m. These separate entry/exit thresholds reduce repeated switching.
- Unknown or partially invalid data in the three front sectors causes a stop.
- Scan validation failures send Stop. A timeout check also sends Stop after more
  than one second of wall-clock time without a scan (checked every 0.1 s).
- Startup and shutdown send Stop. A lock serializes callback and timeout commands;
  callbacks arriving after shutdown cannot restart movement.

`main()` owns the live subscriber. Importing the navigation module no longer
starts a controller. Existing movement helpers in `movement.py` are unchanged.
The navigation imports `OBSTACLE_DISTANCE` from `src/perception/lidar.py`.
`FRONT_CLEAR_DISTANCE = OBSTACLE_DISTANCE + 0.3` therefore now evaluates to 1.3 m.
The diagonal constants remain `DIAGONAL_STOP_DISTANCE = 0.6` and
`DIAGONAL_CLEAR_DISTANCE = 0.8` in `src/navigation/navigation.py`.
Distances refer to the smallest valid scanner-to-surface range in each sector.
Restart the Python controller after editing these constants; Gazebo does not need
restarting for a change to these constants. Navigation needs no new dependency.

Start from the repository root in the owner's working Gazebo Python environment:

```bash
python3 -m src.navigation.navigation
```

Run only one navigation process and avoid concurrent manual velocity publishers
while testing. With Gazebo running, observe forward motion, in-place avoidance,
a consistent turn direction, then resumed forward motion. Ctrl+C sends Stop.

Navigation review for Session 3:

- Retained turn direction and 1.0/1.3-m front hysteresis are consistent. Diagonal
  entry/release thresholds remain 0.6/0.8 m. No speed or steering change was made.
- `analyze_scan()` removes NaN, negative infinity and other invalid ranges;
  `choose_motion()` stops for unknown/partially invalid front sectors. Positive
  infinity is a valid no-return reading. Tests cover these cases and recovery.
- Callback, timeout and shutdown share a command lock. The shutdown flag prevents
  late callbacks from moving the robot. The wall-clock timeout also works when
  simulation time is paused. It measures receipt time, not sensor timestamp age.
- Concrete fix: callback terminal output previously held the command lock. A
  blocked terminal/pipe could prevent the watchdog from sending Stop. Output now
  happens after releasing that lock. Shutdown sends Stop before printing its
  final message. A regression test verifies that neither valid-scan nor
  invalid-scan output holds the command lock.
- This remains reactive sector-based avoidance. It does not check the full swept
  footprint, monitor rear/side clearance during rotation or guarantee escape from
  tight corners. Retaining direction can keep it turning in a trap. No recovery
  planner or footprint model was introduced because the current controller works
  in the owner's test world and such changes require separate tuning/testing.
- The Python watchdog cannot stop a force-killed/frozen controller or replace a
  broken transport connection. Gazebo retains the last command until replaced.
  Queued old scans are not rejected by their simulation timestamp. No simulator
  command watchdog or scan-age synchronization was added in Session 3.

### Session 3: localization and path tracking

Inspected on this Mac: Gazebo Sim **8.15.0** (Harmonic), `gz.transport13` and
`gz.msgs10`. The running server advertises:

    /world/basic_world/dynamic_pose/info    gz.msgs.Pose_V

The world's existing SceneBroadcaster publishes this topic. No simulation or
model file was changed and no additional PosePublisher plugin is required.
`Pose_V.pose` contains several named entities. The extractor selects exactly
`curious_robot`, not `chassis` or a wheel, and rejects duplicate matches. In this
world the robot is a top-level model, so this entry is its world pose; child-link
entries are relative to their parents. This assumption must be revisited if the
robot is later nested inside another model. The DiffDrive odometry topic is not
used as a substitute for world coordinates.

Modules:

- `pose.py`: immutable `Pose2D(timestamp, x, y, yaw)`, quaternion conversion and
  model extraction; no Gazebo imports. Missing fields, nonfinite coordinates,
  invalid time and zero/invalid quaternions are rejected.
- `path.py`: `PathTracker` samples simulation time, retaining only the latest pose
  and last sample. CSV helpers write/read `timestamp,x,y,yaw`. No Gazebo imports.
- `record.py`: subscribes using `Node.subscribe(Pose_V, topic, callback)`, writes
  accepted samples directly to CSV and displays current coordinates, heading,
  simulation time, reception age and invalid-message count. Sends no velocity.
- `plot.py`: offline Matplotlib X/Y line, green start, red end and 0.3-m final-heading
  arrow; equal axis scale and metres. It imports Matplotlib only when plotting.

Coordinates and orientation:

- `x = model_pose.position.x`, `y = model_pose.position.y`, in world metres.
- Timestamp is `message.header.stamp.sec + nsec * 1e-9`, in simulation seconds.
- Normalize quaternion `(qx, qy, qz, qw)` before computing:
  `yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))`.
- Yaw is radians in [-pi, pi], positive counterclockwise around world +Z.
  Heading 0 points along world +X; +pi/2 points along world +Y. Debug output
  additionally converts it to degrees. This is the model origin, not the offset
  LiDAR origin and not an odometry frame.

Recording behavior:

- Default interval: 0.2 simulation seconds (at most approximately five samples
  per simulation second). First valid point is saved immediately. On normal
  shutdown the latest unsampled point is also saved.
- Translation is not required: turning in place records changed headings.
- Equal timestamps, e.g. repeated paused-world messages, do not add points.
- Invalid poses are counted and skipped; absent model entries are ignored. The
  display's reception age and simulation timestamp help identify stalled data.
- Backward simulation time, including a world reset, ends the recording with an
  error and preserves previously written samples. Start a new file afterwards.
- CSV is flushed after each sampled point; RAM usage does not grow with a trip.
  Disk usage grows at the sampled rate. Plotting loads that file into memory.
- Existing CSV files are never overwritten. The default name includes date/time;
  `--output` can specify a different path. `--duration` is wall-clock seconds.
- Ctrl+C closes/unsubscribes cleanly. Stopping the recorder does **not** stop the
  navigation process. No valid poses produces an error and a header-only CSV.

Environment and commands (from the repository root):

A local ignored `.venv` was created using Homebrew Python 3.14.7 with
`--system-site-packages`, so it can use the installed Python-3.14 Gazebo bindings.
Protobuf 7.36.2 and Matplotlib 3.11.2 plus Matplotlib's required dependencies were
installed into that venv. No global package or shell configuration was changed.
The generated installed gz-msgs files require a compatible Protobuf runtime.
The default pyenv Python 3.11.8 still lacks `gz`; use the venv for live commands.

```bash
source .venv/bin/activate
python -m unittest discover -s tests -v

# Separate terminal: only one navigation controller at a time
python -m src.navigation.navigation

# Another terminal: receive and record independently (Ctrl+C or --duration)
python -m src.localization.record --output recordings/run.csv --duration 60

# After recording, open the plot (or export without a GUI)
python -m src.localization.plot recordings/run.csv
python -m src.localization.plot recordings/run.csv --output recordings/run.png --no-show
```

Gazebo server and GUI still start separately as in section 4. Press Run before
expecting changing simulation timestamps. `README.md` contains the full startup
procedure and environment recreation commands. A repeated `run.csv` command
requires a new filename; omitting `--output` creates one automatically.

Automated verification:

- **48 tests pass**: 10 perception, 15 navigation, 21 localization/recording and
  2 Matplotlib plot tests. The 46 non-plot tests need only the standard library;
  none requires a running Gazebo instance. Plot tests skip if Matplotlib is absent.
- Localization tests cover 0/+90/-90/180-degree headings, roll/pitch, normalized
  and equivalent negative quaternions, model selection, timestamps, missing and
  invalid data, sampling, in-place rotation, duplicate/reset times, final points,
  CSV round-trip/rejection, mocked subscription/recovery/shutdown and overwrite
  protection. Plot tests check coordinates, markers, heading, scale and PNG export.
- The baseline had 23 tests with five failures caused by stale perception
  expectations for the former 2.5-m threshold. These are now aligned with 1.0 m.

Live verification and remaining manual checks:

- Python successfully received the real model pose from the owner's existing
  paused Gazebo instance. Repeated simulation timestamp 973.564 produced exactly
  one CSV point, as intended; no commands were sent to that instance.
- A separate server used the unmodified world/model and a dedicated Gazebo
  Transport partition. The modified navigation ran there alongside the recorder.
  A 25-second wall-clock run saved 123 points over 24.869 simulation seconds,
  moving approximately 3.579 m along +X. All recorded poses were valid.
- A second 18-second run saved 88 points over 17.785 simulation seconds, including
  left-turn avoidance and resumed forward motion. Final heading was approximately
  +94.6 degrees and final world position (4.135, 1.811) m. Command logs contained
  137 forward and 33 turning decisions. A real Twist subscriber verified that
  navigation shutdown published (linear.x=0, angular.z=0).
- Real CSV files were loaded and rendered to PNG. The turn-run plot was visually
  inspected for trajectory, start/end, units and final heading. A review copy is
  in ignored `recordings/session3-validation.csv` and `.png`.
- Both test navigation processes and the separate server exited normally. The
  owner's server/GUI/controller were not restarted or controlled. The new lock
  behavior is unit-tested; these short live runs are numerical checks, not a
  complete visual collision or long-duration safety validation.

The owner should still visually compare world position/heading with the GUI,
inspect a longer recorded drive, and check close corners, pause/resume timeout
and Ctrl+C stopping in their normal session. Unit tests and a short numerical
live run do not prove collision freedom for arbitrary obstacle layouts.

### Existing simulation and raw-data test reference

The following commands remain available for inspecting the working simulation.
Stop the old server and GUI before restarting;
keep only one server and one GUI for the normal session. Start them separately
as described in section 4, then press Run.

Terminal 3:

```bash
# List topics and confirm the message type
gz topic -l
gz topic -i -t /model/curious_robot/lidar

# One scan, then continuous scans (Ctrl+C stops the echo)
gz topic -e -t /model/curious_robot/lidar -n 1
gz topic -e -t /model/curious_robot/lidar
```

Each `ranges` entry is a distance in metres from the scanner along one ray.
Using zero-based indexing, ray i has angle `angle_min + i * angle_step`.
Angles are radians relative to the scanner: 0 is forward, positive is left.
`angle_min` / `angle_max` bound the scan; `angle_step` is the angular spacing.
`range_min` / `range_max` bound measurable distances. An infinite range means
no return within the measurable range, not an obstacle at zero distance.
The direction indices and sector slices are listed above.
The single vertical slice may report `vertical_angle_step: nan`; the horizontal
angle step and range data are valid, and no vertical stepping is needed.

At the initial pose, measured examples were approximately:

- front wall: 4.775 m (wall inner face x = 4.9 minus sensor x = 0.125)
- green obstacle at -60 degrees: 1.266 m
- red obstacle at +50 degrees: 1.955 m
- blue obstacle at +150 degrees: 1.871 m

For a short motion test, stop the echo with Ctrl+C and publish a command:

```bash
# Forward at 0.15 m/s
gz topic -t /model/curious_robot/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.15}, angular: {z: 0.0}'

# Stop (the last velocity command persists until replaced)
gz topic -t /model/curious_robot/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}, angular: {z: 0.0}'
```

After 1–2 seconds, send Stop and echo another scan to compare distances.
For a rotation test, publish:

```bash
gz topic -t /model/curious_robot/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.0}, angular: {z: 0.40}'
```

Send Stop after 1–2 seconds, then inspect another scan. Alternatively, leave the
echo running in Terminal 3 and publish movement commands from Terminal 4.

Optional GUI display: open the top-right plugin menu, add **Visualize Lidar**,
refresh its topic list, choose `/model/curious_robot/lidar`, and enable its display.
Numerical scans work without this GUI plugin.

Validation: both SDF files pass `gz sdf -k` (set `SDF_PATH` to `simulation/robot`
when validating the world). The server initializes the sensor on macOS without
a GUI, and `gz topic` receives 361 ranges per scan. Moving forward changed the
front-wall range from about 4.775 to 4.317 m; rotation changed the range pattern.
Test processes used a separate Gazebo Transport partition. No test changes to
obstacle positions were saved in the world file.
The controlled obstacle test returned 1.375 m and 2.375 m when the obstacle
was moved 1 m farther away. Consecutive scan timestamps were 0.1 s apart.
Restarting the sensor and stopping the server with Ctrl+C both succeeded.
The test-only `/server_control` remote shutdown triggered a Gazebo 8.15.0
crash in its stop-event path; use the normal terminal Ctrl+C shutdown.

The previous stop at the raw-LiDAR milestone has been superseded by the owner's
Python perception experiments described above. Mapping, SLAM, ROS 2 and further
sensors are outside the current scope.


## 14. Later Development Roadmap

Possible progression:

### V0.0
Autonomous roaming.

### V0.1
Robot understands / maps places.

### V0.2
Known vs unknown object detection.

### V0.3
Robot asks:
"What is this?"

### V0.4
Persistent object memory.

### V0.5
Robot can ask:
"Is this also a cup?"

### V0.6
Robot actively seeks novelty.

### V0.7
Interests and internal drives.

### V0.8
Episodic memory.

### V0.9
Reinforcement learning for behavior selection.

### V1.0
Integrated curious autonomous agent.


## 15. Future Learning Concept

A likely early object-learning architecture is:

Camera
    ↓
Pretrained Vision Encoder
    ↓
Embedding
    ↓
Similarity Search against Memory
    ↓
Known / Unknown Decision

If unknown:

    Ask Human
        ↓
    Receive Label
        ↓
    Save Label + Embedding
        ↓
    Persistent Memory

Do not initially retrain an entire neural network every time a new object is learned.

Embedding-based memory should be considered first to reduce catastrophic forgetting and simplify incremental learning.


## 16. Future Memory Architecture

Possible memory components:

### Semantic Memory

Knowledge such as:

- this object is a cup
- this room is the kitchen

### Episodic Memory

Experiences such as:

- saw an unknown object near the door
- human identified it as a backpack

### Spatial Memory

Knowledge such as:

- kitchen is north of hallway
- obstacle exists at a certain location


## 17. Future Reinforcement Learning

Reinforcement learning is NOT the first learning mechanism.

RL may later be used for behavior selection, for example deciding whether to:

- explore
- inspect an object
- approach
- retreat
- ask a human
- revisit an interesting location
- recharge

Initial navigation and perception should work without RL first.
