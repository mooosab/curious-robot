# AGENTS.md

## Project

This repository contains the Curious Robot project.

Before making significant changes, read:

    PROJECT_CONTEXT.md

Use it as the primary description of the current project state, architecture and roadmap.


## Current Priority

The current milestone is:

    Session 4 – Spatial Memory / Visited Cells

Continue from the current state described in PROJECT_CONTEXT.md.

The immediate next task is:

    Review the completed Session-4 visited-cell memory implementation.
    Preserve the working simulation and Session-2 navigation.
    Stop for owner review before Session 5; do not merge into main.

Do not jump ahead to AI, reinforcement learning, object recognition or personality systems.


## Development Style

The project owner is learning robotics simulation.

Therefore:

- work incrementally
- prefer simple implementations
- explain important robotics concepts
- avoid unnecessary abstractions
- avoid large rewrites
- preserve working functionality
- make one logical change at a time
- test after meaningful changes


## Before Editing

Before changing a file:

1. inspect the existing repository
2. read the relevant existing files
3. understand the current implementation
4. preserve existing working behavior

Do not replace working files blindly.


## Gazebo

The project currently uses Gazebo Harmonic on macOS.

Gazebo server and GUI are run separately.

Server:

    gz sim -s simulation/worlds/basic_world.sdf

GUI:

    gz sim -g

Custom Gazebo models are currently located under:

    simulation/robot/

The resource path may need:

    export GZ_SIM_RESOURCE_PATH="$PWD/simulation/robot:$GZ_SIM_RESOURCE_PATH"


## Robot

The robot is intended to use differential drive.

Current main components:

- chassis
- left wheel
- right wheel
- left wheel joint
- right wheel joint

A passive caster/support, differential drive and LiDAR are already implemented.
The owner has confirmed stable autonomous driving.


## Spatial Memory Boundary

Spatial Memory reuses the existing Localization pose extraction and runs in a
separate process. Navigation must not read Memory or use visited cells to select
movement unless the owner explicitly requests a later development session.
Visited cells are not an occupancy map or a proof of free space.


## Coding Guidelines

For Python:

- prefer readable beginner-friendly Python
- use descriptive variable names
- avoid unnecessary frameworks
- keep modules focused
- add comments where behavior is not obvious
- avoid premature optimization

For SDF/XML:

- keep formatting consistent
- use descriptive names
- group related robot components
- add short comments for major sections


## Scope Control

Do not introduce a technology simply because it might be useful later.

In particular, do not currently introduce:

- reinforcement learning
- neural networks
- LLM APIs
- databases
- complex distributed architecture
- cloud infrastructure

unless explicitly requested.


## Testing

After modifying the simulation:

- check that the SDF is valid
- check that the world starts
- check that the robot spawns
- check that physics behaves plausibly
- check for obvious collisions or unstable behavior

When implementing movement:

- test forward motion first
- then turning
- then rotation
- only then add autonomous navigation


## Git

Current feature branch:

    feature/session-04-spatial-memory

Keep commits focused and understandable.

Do not commit generated Gazebo/build/cache files.


## Communication

When proposing a change:

1. briefly explain what will change
2. explain why it is necessary
3. implement the smallest useful change
4. explain how to test it

When something fails, diagnose the current failure before adding more features.
