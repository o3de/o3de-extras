## Design document: Vehicle Dynamics for robot control in O3DE with ROS 2 Gem

### Rationale

Many simulated robots need to move around in their environment. 
Whether they drive, walk or crawl, their movement needs to be similar to how a real robot behaves.
Depending on the purpose of simulation for a given use-case, a rough approximation of the actual movement might be enough.
In other cases, the robot dynamics and its interaction with the environment through movement is the key property to validate.

Much of the work that is required to create a dynamics model for variety of simulated robots is common and can be 
abstracted into convenient components and tooling.

### Purpose

To support developers of robotic use-cases in O3DE, a modular, extendable system for simulation of vehicle dynamics is required.

### Subsections

- [Analysis](analysis.md)
- [Features](features.md)
- [Architecture](architecture.md)

### Note

| In the first iteration, only support for car-like 4-wheeled robots will be targeted. |
|--------------------------------------------------------------------------------------|
