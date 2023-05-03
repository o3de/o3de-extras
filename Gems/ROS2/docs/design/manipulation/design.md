## Design document: Manipulation for robots in O3DE with ROS 2 Gem

### Rationale

One the most used features in the robotics worlds are the interaction of the
robots with the objects that exists in the real life.

The manipulation act is composed by many different areas that are themselves
large research topics such as perception, planning and control. From the physic
point of view the robot manipulation is generally done by a series of actuators
usually connected each other in many different ways and a good variety of end
effectors to support operations on the desired objects.

A good part of the manipulation work load is designed to deal with kinematics
and dynamics constraints imposed by the physical design of the robots and the
real world physics properties.

This design must consider the connection from O3DE to third party libraries and
frameworks that already implement large part of the topics listed above and do
not try to reimplement them although native capabilities can be consider in a
case by case basics.

### Purpose

To support developers of robotic use-cases in O3DE, a modular, extendable system for robot manipulation
is required.

### Subsections

 - [Analysis](analysis.md)
 - [Features](features.md)
 - [Architecture](architecture.md)
 - [Implementation Roadmap](roadmap.md)

### Notes

 * The design does not consider any topic related to perception.
 * The design will prioritize the connection with the usual ROS 2 frameworks and tools
 * The design does not consider any grasping technique although external frameworks can provide this
   capability.

| In the first iteration, only support for car-like 4-wheeled robots will be targeted. |
|--------------------------------------------------------------------------------------|

