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
- Python for initial LiDAR perception experiments
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

    feature/session-01-basic-simulation

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
    │   ├── navigation/
    │   ├── perception/
    │   │   └── test.py
    │   ├── memory/
    │   └── brain/
    │
    ├── config/
    └── tests/

`src/perception/test.py` is the current LiDAR reception experiment.
The `navigation`, `memory`, and `brain` directories exist but are empty;
`config` and `tests` are also empty. No Python navigation, Brain or Memory
implementation has been started.


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

We are currently working on:

    Session 1 – Basic Simulation

The purpose of Session 1 is ONLY to establish the physical robot and basic autonomous navigation.

The simulation hardware is working. The current development focus is learning
to process LiDAR data in Python, progressing from individual directions to
angular sectors. Autonomous navigation remains a later, unstarted step.

Do NOT add:

- machine learning
- reinforcement learning
- object recognition
- camera learning
- LLM integration
- personality
- long-term memory

yet.


## 11. Session 1 Definition of Done

Session 1 is complete when:

1. Gazebo world starts successfully.
2. Differential-drive robot appears correctly.
3. Robot can move using its wheels.
4. LiDAR produces distance measurements.
5. A simple Python controller can avoid obstacles autonomously.


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

Session 1 status: world working, stable robot working, differential drive working,
LiDAR working, and Python LiDAR subscription and conversion to a list working.
The owner has confirmed reception through the Python bindings. The current saved
script extracts and prints eight individual directions. Angular-sector processing
is in progress and is not complete; see section 13 for the exact state.
Python navigation and autonomous obstacle avoidance have not been started.
Brain and Memory have not been started. ROS 2 is not in use.


## 13. NEXT STEP

### Current perception state

`src/perception/test.py` imports `Node` from `gz.transport13` and `LaserScan`
from `gz.msgs10.laserscan_pb2`. It subscribes to `/model/curious_robot/lidar`
with `node.subscribe(LaserScan, "/model/curious_robot/lidar", lidar_callback)`.
The callback receives the LaserScan data and converts its 361 ranges into a
normal Python list with `lidar_data = list(msg.ranges)`. A sleeping loop keeps
the subscriber process alive. This is a perception experiment, not a robot
movement controller.

The eight directions and their zero-based indices have been understood and
are read by the current saved script:

| Direction | Index | Angle relative to forward |
|---|---|---|
| Front (vorne) | 180 | 0 degrees |
| Front-left (vorne-links) | 225 | +45 degrees |
| Left (links) | 270 | +90 degrees |
| Back-left (hinten-links) | 315 | +135 degrees |
| Back (hinten) | 0 (also 360) | -180 degrees (also +180 degrees) |
| Back-right (hinten-rechts) | 45 | -135 degrees |
| Right (rechts) | 90 | -90 degrees |
| Front-right (vorne-rechts) | 135 | -45 degrees |

The owner has begun replacing individual directions with angular sectors.
The current front-sector example is `lidar_data[170:191]`: indices 170 through
190 inclusive, covering -10 through +10 degrees around forward (21 values).
This describes the current learning/work-in-progress state reported by the
owner. At this documentation update, the saved `test.py` still uses individual
indices, including `lidar_data[180]`; the sector slice is not yet in that file.

### Next steps — not completed

1. Continue replacing individual direction samples with angular sectors.
2. Process the sector values, for example by determining a minimum with `min()`.
3. Handle measurements robustly, including incomplete/empty data, invalid values
   and non-finite ranges.
4. Develop obstacle detection from the processed sectors.

Sector aggregation, robust measurement handling and obstacle detection are not
finished and are not implemented in the current saved script. Python navigation,
autonomous obstacle avoidance, Brain and Memory remain unstarted. The owner wants
to write and understand the Python code themselves; documentation of these next
steps does not authorize their automatic implementation.

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
The direction-to-index mapping is listed above.
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
