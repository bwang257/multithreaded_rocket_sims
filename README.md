### Multithreaded Dynamic Rocket Sims

**Project Goal**: Simulate faster-than-real time an entire flight to prove how much of an effect errors in any calculations will cause (ex. how off can the aero moment coefficient of the fins be off until we are unstable). 

**Current Task**
Simulate rocket flights in ~10ms each. The rocket current takes 37 seconds in real time to peak, with a total flight of ~5 minutes. 

We want this to be a tool that any controller can deploy to, where controllers are made in MATLAB/Simulink, exported to C code. The plant runs inside this codebase and we want the export from Simulink to running the sims pathway to be as simple as possible? There also exists poor C code exported from Simulink. The fix and approach to that is potentially enforcing that the source code is written in a way that the generated C code is fast. 

Ultimately, the current tasks is to work towards getting the C++ model/dynamics working, understanding how to multithread it, and integrate it with matlab to c code. 

**Eventual Task**
Deploy the simulation on everyone's computers (prioritize linux, followed by windows) and a HITL system. This will be an overnight CI system.

**Background**
A plant refers to the thing being controlled (in this case the rocket's 6 DOF dynamics). A controller is an algorithm that estimates state and commands an actuator. The c++ model that is to be built centers around the plant.

The simulation should take a .c/.h file exported from Simulink. 


### Resources
- [.ork2json](http://github.com/RocketPy-Team/RocketSerializer)
- [rocketpy](https://github.com/RocketPy-Team/RocketPy/tree/master)


