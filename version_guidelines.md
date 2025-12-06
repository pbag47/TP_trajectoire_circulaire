
# Migration to latest updates

The first goal of this branch is to make the code compliant with
newer Python libraries updates.

In addition, a new project structure is implemented, 
having in mind a better integration of virtual vehicles, 
simulations and external rendering programs like Unity or 
Unreal Engine

This new architecture does not bring any major new feature, 
but it is intended to be a development step towards more 
modularity and integration.

## Changes
- Instead of a flat directory, the program structure is now 
articulated in topic-related submodules which bring more 
clarity to the code
- Instead of a bespoke UAV-simulation script, the UAV 
simulation is now integrated well better by taking advantage 
of class-inheritance, so any vehicle type can have its own 
virtual counterpart with simulated behavior
- Even QTM can be simulated (only if all the vehicles are
virtual)
- During the setup, since many changes can occur before
the actual flight begins, 
an abstract vehicle class called VehicleRepresentation
is involved instead of the specific vehicle classes.
The transition from setup to real-time is crucial 
as it must generate the requested vehicle instances 
with the attributes of abstract instances. This transition
is yet to be implemented.
  - This transition is handled by the 
```flight_parameters.txt``` file, which carries information from
setup to real-time phases. This single file is replaced by a set of 
class-specific files in the ```config``` folder
- A kind of "physics engine" based on dynamic fundamentals
is intended to replace the UAV behaviour model: adding external 
forces and perturbations like aerodynamic drag
or wind would then be easier
- Late feature idea: adding "obstacle" object and automatically 
create an obstacle instance when an undeclared QTM marker 
appears