# Selected features and their suggested importance

| Feature | Description | Importance |
| ------- | ----------- | ---------- |
| Modular and configurable design | <ul><li> Highly customizable – can support variety of robots/vehicles </li><li> Modular – easy to replace implementations of parts with alternatives </li><li> Good defaults – less steep learning curve </li></ul> | Crucial |
| Vehicle configuration |  <ul><li> Number and configuration of wheels, axles, etc. </li></ul> | Crucial |
| Core dynamics | <ul><li> Realistically transfer inputs to robot/vehicle behavior</li><li>Multiple steps per simulation step (sub-step solver)</li><li>Considering vehicle parameters such as mass and inertia</li><li>Considering engine/transmission parameters, such as max torque, max angular velocity</li><li>Selection of drive wheels (minimum: front and/or rear)</li></ul> | Crucial |
| Wheels and suspension | <ul><li>Handle ground detection</li><li>Apply friction model (speed dependent)</li><li>Use wheel 3D geometry for collision</li><li> Adjust camber, caster, rim offset </li><li> Handle wheel visuals (rotate, turn)</li><li> Handle spring and damper, adjustable curves </li><li> Type of steering (differential or car-like)</li><li>Selection of steering wheels </li></ul> | Crucial |
| ROS 2 Control | <ul><li> Setting a topic, QoS, handling subscriptions</li><li>Selecting message type for control – AckermanDrive or Twist (see [this discussion](https://discourse.ros.org/t/is-twist-still-a-good-velocity-command-interface/13218/4)) </li><li> Applying control to the robot/vehicle. Stopping on no input. </li></ul>| Crucial |
| Input System |  <ul><li> At least inputs for brakes, acceleration, and steering wheel</li><li> Other inputs such as turn signals, lights, clutch, handbrake etc. </li><li>Connect with a manual control component for testing (inputs can be bound to keys/controls)</li></ul> | Crucial |
| Axles | <ul><li>Set wheels for axle </li><li> Adjust steering (coefficient, Ackermann, toe angle) </li><li> Adjust caster and camber angles Brake and handbrake, adjustable strength </li></ul> | Important |
| Cruise control | <ul><li> Keep a given speed </li></ul> | Important |
| Path following | <ul><li> Follow a given path with a configurable speed limit </li><li> Adjust speed when approaching curves </li><li> Optional: stop when something is in the way (based on ground truth). This could be useful for testing.</li></ul>  | Important |
| Engine | <ul><li> Power curve, losses, limits </li></ul> | Optional |
| Transmission | <ul><li> Automatic, Manual, etc. </li><li> Gears, gear ratios, shifting behavior </li></ul> | Optional |
| Aerodynamics | <ul><li> At least a simple model to calculate aerodynamic drag </li></ul> | Optional |
| Visuals and sounds | <ul><li> Lights </li><li> Engine and other sounds </li><li> Skid marks  </li><li>  Particle effects, exhaust smoke </li></ul> | Optional |
| Transmission | <ul><li> Power distribution between wheels </li></ul> | Optional |
| Axles | <ul><li> Brake force distribution between wheels </li></ul>| Optional |
| Wheels and suspension (additional features) | <ul><li> Steering angle difference/coefficient for each wheel  </li><li> Specify several types of suspension  </li><li> Spring compression/force curve  </li><li> Damper velocity/force curve </li><li> Handle differential drive mechanism and its configuration </li></ul> | Optional |
