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
- Python for LiDAR perception, reactive navigation, localization and visited-cell memory
- Matplotlib for offline trajectory plots
- working Python bindings for `gz.transport13` and `gz.msgs10` (confirmed by the owner)

Planned:

- ROS 2 if appropriate
- computer vision
- pretrained vision embeddings
- semantic / episodic memory
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

    feature/session-05-exploration

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
    │   ├── exploration/
    │   │   ├── __init__.py
    │   │   ├── basic.py
    │   │   └── explore.py
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
    │   │   ├── __init__.py
    │   │   ├── spatial.py
    │   │   └── record.py
    │   └── brain/
    │
    ├── config/
    └── tests/
        ├── test_exploration.py
        ├── test_exploration_live.py
        ├── test_localization.py
        ├── test_localization_plot.py
        ├── test_memory_record.py
        ├── test_spatial_memory.py
        ├── test_navigation.py
        └── test_perception.py

`src/perception/test.py` displays live LiDAR data. Navigation and movement are
implemented in `src/navigation/`; localization records the simulator's model pose
independently in `src/localization/`. `src/memory/` receives that localization
through a separate subscriber and stores visited cells. `brain` and `config`
remain empty. The original navigation module does not consult Spatial Memory.
The Session-5 runner reuses its Safety decision, then selects local directions
using Localization and Memory. Only one driving controller may run at a time.
Tests use `unittest`; the two plot tests use Matplotlib and skip if it is absent.
Local `.venv/`, generated `recordings/` CSV/images and `data/*.json` memory
files (including temporary save files) are ignored by Git.


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

    Session 5 – Basic Exploration

- Session 1: Simulation + Perception.
- Session 2: Safe Navigation (reactive avoidance, with documented limitations).
- Session 3: Localization + Path Tracking using the simulator's world pose.
- Session 4: Visited Cells, geometric coverage and JSON persistence.
- Session 5: Local, deterministic preference for unknown cells among allowed
  directions; Safety always has priority.

Exploration runs as the single driving controller and single writer of its
Memory file. Do not run `navigation.py` or `memory.record` alongside it on the
same robot/file. The independent path recorder may run alongside Exploration.
Stop after Session 5 for owner review. Do not add SLAM, occupancy mapping,
obstacle maps, global goals/planning, frontier exploration, cameras, semantic
memory, RL or other AI. Do not merge main.


## 11. Session 5 Definition of Done

1. Reuse Session-2 Safety, Session-3 localization and Session-4 persistence.
2. Score a few local candidate directions using unique unvisited cells.
3. Keep obstacle/invalid-data/timeout/shutdown handling ahead of Exploration.
4. Update and persist visited cells during driving, with periodic Coverage output.
5. Use a single controller; retain deterministic choices and turn state.
6. Pass all existing tests plus new Exploration and runner tests.
7. Record honest live-test results and known limitations.

Implementation details, test results and live verification are in section 13.


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
recording and plotting. Session 4 adds persistent visited-cell Spatial Memory.
Session 5 adds Basic Exploration without changing the existing perception,
navigation, localization or memory modules. Brain, semantic/episodic memory
and ROS 2 remain unimplemented.


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

- Session-3 validation: **48 tests passed**: 10 perception, 15 navigation, 21 localization/recording and
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

### Session 4: Spatial Memory / Visited Cells

Architecture (no changes to existing localization, perception or navigation):

    Gazebo Pose_V -> localization.extract_model_pose() -> Pose2D.x/y
        -> SpatialMemory.visit() -> set of unique grid cells -> JSON

`src/memory/spatial.py` contains the pure, Gazebo-independent `SpatialMemory`.
`src/memory/record.py` subscribes separately, reusing `extract_model_pose` and
Session 3's `POSE_TOPIC` constant. There is no second pose parser, quaternion
conversion or coordinate system. Navigation neither imports this module nor
reads the memory file. No new dependency was installed for Session 4.

Grid configuration and coverage:

- Default cell size: **0.5 m × 0.5 m**, defined once in `spatial.py`.
- `cell_x = floor(x / cell_size)`, `cell_y = floor(y / cell_size)`.
  The grid origin is world (0, 0). Examples: (-0.1, 0.1) -> (-1, 0),
  (-0.5, 0.5) -> (-1, 1). Truncation toward zero would be wrong for negatives.
- `basic_world.sdf` has wall centers at +/-5 m and wall thickness 0.2 m.
  Interior faces are +/-4.9 m, so the interior rectangle is **9.8 × 9.8 m**.
  Default bounds are `(xmin, xmax, ymin, ymax) = (-4.9, 4.9, -4.9, 4.9)`.
- Lower bounds are inclusive, upper bounds exclusive: `xmin <= x < xmax`,
  `ymin <= y < ymax`. Out-of-bounds positions are rejected rather than clamped.
- `total_cells` counts all origin-aligned grid cells intersecting this rectangle,
  including partial edge cells: indices -10 through 9 on each axis, **400 cells**.
  Partial edge cells count once, just like interior cells; this is a cell-count
  metric, not an area-weighted floor-coverage percentage.
- `coverage = count / total_cells`, always in [0, 1]. Obstacles and robot footprint
  are not subtracted. Thus 100% may be physically unreachable.
- Cell size and bounds are configurable at construction and via runner CLI.
  Public configuration properties and the `visited_cells` frozenset are read-only.

Small API:

```python
from src.memory.spatial import SpatialMemory

memory = SpatialMemory.load("data/spatial_memory.json")  # absent -> empty
memory.world_to_cell(-0.1, 0.1)  # (-1, 0)
memory.visit(-0.1, 0.1)         # True only for a newly visited cell
memory.was_visited(-0.2, 0.2)   # True
memory.visit_cell(2, 3)
memory.was_cell_visited(2, 3)
print(memory.count, memory.total_cells, memory.coverage)
memory.save("data/spatial_memory.json")
```

Persistence format (example, not preloaded demo data):

```json
{
  "format": "curious-robot-visited-cells",
  "version": 1,
  "cell_size": 0.5,
  "bounds": [-4.9, 4.9, -4.9, 4.9],
  "visited_cells": [[-1, 0], [2, 3]]
}
```

- Default path: `data/spatial_memory.json`, relative to the repository root.
- `load()` returns a new object. Missing files yield an empty memory without
  creating a file. Invalid JSON, duplicate fields/cells, unsupported versions,
  noninteger/out-of-grid cells and invalid configuration raise a visible error.
- Saved cell size/bounds must match the requested configuration. For a custom
  file, pass the same configuration to `load()` or the CLI; existing cells are
  never silently reinterpreted. Loading errors do not overwrite the file.
- `save()` writes sorted cells to a temporary file in the target directory,
  flushes/fsyncs and atomically replaces the destination. A failed replacement
  leaves the old JSON intact and removes the temporary file.
- Only one writer per memory file is supported. Different worlds/experiments
  should use separate files; equal bounds do not establish world identity.

Standalone Memory use with the original reactive controller only (do not also
write the Exploration controller's file). Gazebo server/GUI start separately as
in section 4; press Run:

```bash
# Separate terminal, parallel to the existing navigation process
source .venv/bin/activate
python -m src.memory.record

# Optional bounded run or separate experiment file
python -m src.memory.record --file data/experiment.json --duration 60

# Inspect persistent data, then run the same command again to resume
python -m json.tool data/spatial_memory.json
```

Each valid received model pose marks its current cell, including poses with a
repeated or reset simulation timestamp. Repeated visits cause no duplicates.
Invalid/out-of-bounds poses are counted and skipped. Missing model entries are
ignored. Status is printed every two wall-clock seconds: unique cells, coverage,
reception age/waiting state and rejected-pose count. No per-pose terminal spam.

Disk saves occur at most every five wall-clock seconds (`--save-interval`), only
when new cells have appeared or a new memory file needs initialization. Ctrl+C
or `--duration` expiry saves pending changes and unsubscribes. If no pose arrives,
the runner reports that explicitly and preserves previously visited cells.
Stopping Memory does not stop Navigation. Hard termination may lose unsaved
recent visits; no multi-process merging or database is implemented.

Known limitations: visits refer to the model origin, not the full robot footprint
or LiDAR-visible area. Cells between missing pose messages are not interpolated.
Position jitter at a cell boundary can mark adjacent cells. A world reset retains
visits intentionally; choose a new file for a fresh experiment. Neither obstacles,
free space, reachability nor exploration targets are inferred from this set.

Session-4 verification:

- Session-4 validation: **78 tests passed**, including all 48 existing tests and 30 new tests (21 grid/JSON
  and 9 runner tests). The two existing Matplotlib tests skip if it is absent;
  all other tests use the standard library and need no running Gazebo.
- Tests cover positive/negative coordinates, same/different cells, duplicates,
  direct cell operations, immutable public configuration, coverage up to 100%,
  partial/exact boundaries, invalid data, custom configuration, JSON round-trip,
  missing/corrupt/incompatible files, atomic-save failure, mocked live reception,
  periodic dirty saves, Ctrl+C persistence, restart, stale callbacks, reset times,
  failed subscriptions and invalid intervals. A test compares the default bounds
  with the actual SDF wall collision geometry.
- Run all tests: `python -m unittest discover -s tests -v`.
- Run only Session 4: `python -m unittest discover -s tests -p 'test_*memory*.py' -v`.

Live validation (separate Gazebo Transport partition, unmodified basic_world):

- A fresh server ran with the existing navigation while Memory recorded poses.
  The initial 15-second run saved six cells (1.50% coverage).
- Restarting the Memory process on the same JSON loaded all six cells, then
  extended the set to eight cells (2.00%) during a further six seconds of driving.
  JSON inspection confirmed that every previous cell remained present. Both
  Memory runs and the navigation process exited with code 0.
- A separate real Ctrl+C test interrupted Memory before the five-second periodic
  save. No file existed before the signal; shutdown wrote the current visited
  cell and exited with code 0.
- Gazebo initially printed multicast "No route to host" warnings, but topic
  discovery and Python reception subsequently worked without environment changes.
  All test processes were stopped; normal user simulation sessions were untouched.
- Tests used temporary JSON files, not the user's default memory. No preset
  visited cells were added to the repository. Startup jitter near world y=0 marked
  both adjacent rows, illustrating the documented boundary-noise limitation.

Manual follow-up: run Memory alongside one navigation controller, observe count
and coverage, Ctrl+C only Memory, inspect the JSON, then restart it with the same
file and verify the initial loaded count. Continue driving to observe additions.
This verifies persistence; it does not validate collision avoidance or coverage
of physically unreachable cells.

### Session 5: Basic Exploration

Architecture:

    LiDAR -> existing analyze_scan -> existing choose_motion (Safety first)
    Pose_V -> existing extract_model_pose -> current x/y/yaw
    x/y -> existing SpatialMemory.visit -> persistent visited cells
    Safety + Pose + Memory -> BasicExplorer -> existing movement.move

`src/exploration/basic.py` contains pure geometry/scoring and a small stateful
`BasicExplorer`. `src/exploration/explore.py` is the single driving runner.
No existing production module or old test was changed for Session 5. No package
was installed. Session-4 JSON version 1 and its binary visited/unvisited set remain
unchanged; there are no visit counts or rewards.

Local scoring, configured centrally in `basic.py`:

- Candidate relative headings: **0°, +45°, -45°**, aligned with the front and
  front-diagonal LiDAR sectors. No global search or distant goal selection.
- Lookahead distances: **0.5, 1.0, 1.5 m** from the model origin.
- `angle = normalize(yaw + relative_angle)` into [-pi, pi).
  `x_target = x + d*cos(angle)`, `y_target = y + d*sin(angle)`.
  Trigonometric round-off near exact cardinal axes is snapped to zero before
  projection, preventing `sin(pi)` residue from assigning the wrong grid row.
- Convert targets using the existing `world_to_cell`. Stop at the configured
  world boundary; deduplicate cells and exclude the robot's current cell.
- Do not score beyond an observed obstacle: only distances satisfying
  `d + 0.45 <= candidate_sector_min_range` are used. The 0.45-m margin covers
  approximately 0.325-m model footprint radius plus the 0.125-m LiDAR offset.
- **Score = number of distinct unvisited retained cells**. Visited cells score 0.
  Memory is not treated as evidence of free space. No distance weights/randomness.
- Highest score wins. Equal scores prefer straight, then left, then right.
  With no positive novelty gain, no voluntary turn is introduced. During a
  necessary Safety maneuver, a known allowed side may still be selected.

Safety and state:

- Each command first calls the unchanged `choose_motion()`. Its 1.0-m front
  threshold, 0.6-m diagonal entry threshold, 1.3/0.8-m release hysteresis,
  0.15-m/s forward speed and +/-0.5-rad/s rotation remain in effect.
- Voluntary candidate headings require valid center/adjacent sectors and the
  stricter existing release distances: center >=1.3 m, adjacent sectors >=0.8 m.
- Every rotation additionally requires **all eight sectors valid and >=0.70 m**.
  This conservative guard is specific to the new runner. The chassis's maximum
  radius around the wheel axle is `hypot(0.35,0.20)=0.403 m`; the LiDAR is 0.225 m
  from that axle. Their sum is 0.628 m; 0.70 m adds reserve. This may stop the
  robot in tight places where the old controller would continue turning.
- Once obstacle avoidance begins, its chosen direction is retained until the
  original hysteresis releases it. Memory can choose a side only at the start
  of the maneuver, among allowed candidates. If there is no candidate, the old
  in-place avoidance direction is used only if the all-around guard permits it;
  otherwise Stop. Exploration never introduces forward motion during avoidance.
- A voluntary turn stores a short heading target (not a position goal), rotates
  in place and retains it until within **5°**. Safety may interrupt it at any time.
  The target sector and all-around clearance are rechecked during the turn.
- Reconsider voluntary direction after **0.5 m** of position progress. This lets
  the robot enter new cells instead of endlessly rescoring while stationary.
  After avoidance/turn completion it also gets this short forward commitment,
  always subject to fresh Safety checks. A 0.5-m forward projection must remain
  within Memory bounds; otherwise choose an allowed turn or Stop.

Runner data freshness and concurrency:

- `ExplorationSession` holds sensor state, Memory and command state under one lock.
  Both callbacks and commands use it; shutdown disables late callbacks.
- A separate control thread runs every **0.1 wall-clock seconds**. It stops if
  either sensor is absent/invalid, Scan age >1 s, Pose age >1 s, or the difference
  between their simulation timestamps exceeds **0.25 s**. Repeated timestamps
  do not renew reception freshness. Pausing the simulation therefore stops motion.
- Backward timestamps stop the controller with an error; restart it after resetting
  Gazebo. Unlike the passive Session-4 recorder, a driving controller must not
  retain old heading decisions across a reset.
- JSON saves and status prints are outside the command lock and control thread.
  A snapshot is saved every five wall-clock seconds only if the visited count
  changed. Newer visits remain eligible for the next save. Shutdown stops first,
  then unsubscribes and saves any remaining visits.
- Callback parsing errors invalidate cached data; unexpected control-thread errors
  stop movement and are surfaced by the runner. Initial/shutdown commands are zero.
- Existing JSON is loaded before startup; corruption/configuration mismatch fails
  visibly without replacing the file. Only one process may write that file.

Usage (from repository root; server and GUI still start separately):

```bash
source .venv/bin/activate
# First stop the old navigation controller and any Memory writer with Ctrl+C.
python -m src.exploration.explore

# Separate experiment file and bounded simulation duration (plus wall-clock cap)
python -m src.exploration.explore --file data/exploration_trial.json --sim-duration 60 --duration 120

# Inspect saved visits; restart with the same file to retain knowledge.
python -m json.tool data/exploration_trial.json
```

The default file remains `data/spatial_memory.json`. `--cell-size` and `--bounds`
must match a pre-existing file. `--sim-duration` measures from the first received
pose, `--duration` is wall-clock time. Status every two seconds shows elapsed
simulation time, visited-cell count, Coverage and current decision reason.
The standalone Memory recorder is unnecessary while Exploration runs.
The old `python -m src.navigation.navigation` remains available as an alternative,
never as a parallel controller. To compare, use fresh identical start worlds and
separate initially identical memory files; report equal simulation durations.

Testing:

- **118 tests pass**: all 78 existing tests unchanged, plus 24 pure Exploration
  and 16 session/runner tests. The existing two Matplotlib tests remain optional.
- New tests cover projection/wrapping/cardinal and negative coordinates, bounds,
  deduplication, unseen/partially seen/known scores, deterministic ties, Memory
  changing a repeated route, blocked unknown versus allowed known directions,
  no-return/invalid data, hysteresis/direction retention, target commitment,
  side/rear rotation clearance, timeouts, repeated/mismatched/reset timestamps,
  Memory updates/snapshots, loading/saving, Ctrl+C/late callbacks and bad files.
- Run: `python -m unittest discover -s tests -v`.
- Exploration only: `python -m unittest discover -s tests -p 'test_exploration*.py' -v`.

Live verification used real Gazebo physics and transport in separate partitions.
World/model repository files were untouched. Temporary world copies changed only
spawn poses for targeted checks; no test object positions were saved in the repo.

- A first integrated run completed 60.0 simulation seconds and saved 20 cells
  (5.00%). An independent Session-3 path recorder captured the driven trajectory.
- Comparison from the same start pose and empty memory used the same passive
  pose observer for both controllers. Pose streams were trimmed to a nominal
  60-second window from their first observation (last samples at 59.985 and
  59.999 seconds due to topic sampling):

  | Controller | Visited cells | Coverage |
  |---|---:|---:|
  | Existing reactive navigation | 18 | 4.50% |
  | Basic Exploration | 20 | 5.00% |

  This single comparison includes startup waiting for both controllers and is
  only a sanity check. It is not proof of generally better coverage or fewer loops.
- Approximately 10-s trials started near a wall (3.7, 0, yaw 0), a corner
  (3.6, 3.6, yaw pi/4), the red box (0.25, 2, yaw 0), and a close front-diagonal
  green box (0.15, -0.6, yaw 0). Wall/corner/box trials drove/turned; the close
  diagonal trial deliberately stayed stopped under the rotation-clearance guard.
- At received poses, projected chassis/wheel/support bounding rectangles were
  checked against world wall/box rectangles. No overlaps were detected. Minimum
  projected separations were approximately 0.609 m (wall), 0.624 m (corner),
  0.357 m (box) and 0.300 m (stopped diagonal case). The comparison runs also
  showed no projected overlap (reactive minimum 0.407 m; Exploration 0.610 m).
  This is sampled planar geometry validation, not force/contact-sensor evidence
  or a guarantee for all obstacles, dynamics and simulator timings.
- A restart from the initial pose loaded the first run's actual 20-cell JSON.
  All 20 cells remained present. Within the short repeat trial the controller
  turned left to about +42.5 degrees, reaching approximately (0.907, 0.456) m;
  the fresh-memory run initially continued straight. It had not yet entered a
  new cell by the repeat's end, so coverage stayed 5.00%. The comparison therefore
  demonstrates changed behavior from retained knowledge, not immediate gain.
- All test controllers exited with code 0 after normal duration expiry or SIGINT.
  Real Twist subscribers saw final (0,0) commands in the comparison and targeted
  trials. All test servers/controllers were stopped afterwards. No user's default
  JSON was overwritten. Unit tests additionally cover missing/invalid/old sensors.

Owner follow-up: visually check a longer drive, pause/resume and Ctrl+C, close
corners/diagonals, and restart with the same Memory file. Run no competing
controller or writer. Tight-clearance stops are expected, and there is no
automatic escape maneuver. Full safety over arbitrary layouts is not established.

Limitations: this remains a local heuristic, not collision-free path planning.
A sector minimum and a planar LiDAR cannot certify all 3D swept geometry or
moving obstacles. The conservative rotation guard can leave the robot stopped
near walls/diagonals. No escape/reversing planner is added. When all nearby cells
are known, deterministic reactive routes/loops may persist. Boundary jitter,
missing pose samples and unreachable cells retain Session 4's limitations.
Coverage does not prove obstacle-free or physically complete exploration.
Only one controller/writer is an operating requirement; legacy scripts do not
share a cross-process lock. A force-killed/frozen process or broken transport
still cannot guarantee a final Stop command in the simulator.

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
