# Metamorphic Testing in Driving Simulators

Metamorphic Testing is a unique approach to testing software that leverages the relations between inputs and outputs across multiple executions, termed as Metamorphic Relations (MRs). This document aims to outline MRs that can be applied in the context of testing driving simulators.

## 1. Spawning Actors
We can monitor the outputs by the collision results, and location data after spawned.

**The code does not allow to spawn two actors at the same or very close location to avoid collision.**

**Bug Found:** By running the iteration loop of trying to spawn the actor near the ego vehicle, I have identified a problem when try to spawn actors on the ego vehicle where should not be allowed to spawn.
**Issue Ticket**: [
Bug Report: The Spwan Function has permitted to spawn the actors at the location which will cause collisions
](https://github.com/carla-simulator/carla/issues/6653)

- **Scenario 1**: Two actors spawned at the same position should either collide immediately or occupy the same space.
  - The scenario cannot be loaded. 
  - #### MRP<sub>FollowDirection</sub>:
    - Spawn a third actor around the two actors, should not affect the situation of the two actors. E.g., one actor may fly because of the collision, then adding a third non-contact actor should not affect the track of "fly".
  - #### MRP<sub>PropertyVariations</sub>:
    - Rotate both actors without changing the spawn location should give the same results. 
 
  - #### MRP<sub>CombineScenarios</sub>:
    - Scenario 1: One cyclist spawned in front of the ego vehicle (both bboxes crossed)
    - Scenario 2: Another actor vehicle spawned behind the ego vehicle (both bboxes crossed)
    - Scenario 1 + 2: the data of ego vehicle should not change (?)

  


## 2. Changing Environments

- **Change Traffic Lights:** Change the traffic light from red to other colors regularly (Vehicles will only be aware of a traffic light if the light is red.)

- **MR1**: Changing the environment (e.g., from dry to rainy) should cause the vehicle's behavior to change accordingly.
- **MR2**: The vehicle's behavior should revert back to its original state when the environment is changed back to the original state.
- **MR3**: If the environment changes from one state to another and back again, the behavior of the vehicle should revert back to the original state.
- **MR4**: Changing the environment while the vehicle is stationary should not affect the vehicle's position.

## 3. Control Movements

- **MR1**: If the same control movement is input twice in succession, the resulting direction of movement or position should be more to the left compared to just one input (if the control was to steer left, for example).
- **MR2**: If a control movement and then the opposite control movement are performed, the resulting direction of movement or position should be close to the original position before the movements.
- **MR3**: If no control movements are input, the vehicle should continue in its current state.
- **MR4**: If the same control movement is input after a delay, the vehicle's state should change as if the delay did not happen.

- **Bug Found:** The CheckMaximumVelocity Criteria gave wrong value when pushing the throlttle to the limit for ten seconds since the simulation starts. 
  - [Bug Report: Class CheckMaximumVelocity in Scenario Runner Gives False Values #1009](https://github.com/carla-simulator/scenario_runner/issues/1009)

## 4. Vehicle Dynamics

Control the movement of the vehicle by code.

- **MR0**: When the ego starts, start throttle then changing the steering will cause the GNSS data different to first change steering then starting throttle.
  - In different seconds the results should all be different.
  - NO MR Violations found.
  
- **MR1**: If the same force is applied twice to a vehicle from rest, the resulting speed should be approximately double compared to applying the force once.
- 
- **MR2**: If a force and then the opposite force are applied to a moving vehicle, the resulting speed should be close to the original speed before the forces were applied.
  - **The following relationships should hold:**
    1. speed[[w,3]] >= speed[[w,1],[w,1],[w,1]] 
    2. speed[[w,3],[s,1],[s,1]] >= speed[[w,3],[s,2]]
    2.1. speed[[w,4],[s,1],[s,1]] >= speed[[w,4],[s,2]]
    
    3. speed[[w,3],[s,1],[s,1]] >= speed[[w,1],[w,1],[w,1],[s,1],[s,1]]

  - **Bug FOUND:** [Abnormal Phenomena Relating to the Vehicle Dynamics of Tesla Model 3 (vehicle.tesla.model3)](https://github.com/carla-simulator/carla/issues/6673)

- **MR3**: If the same force is applied to a moving vehicle twice in succession, the resulting speed decrease should be approximately double compared to applying the force once.
- **MR4**: If a vehicle in motion has a force applied until it stops, applying the same force again should not change the vehicle's state.


## 5. Simulator Performance
- Are there anything to trigger the infinite loop of the simualtor that consume all the resources and make the software quit unexpectedly (maybe caused by the behavor of actors)

1. **Large Scale Simulations**: Try creating a city-scale simulation with a large number of different actor types (e.g., cars, pedestrians, traffic lights). Then, make them interact in complex ways. CARLA should be able to handle it, but pushing it to the limit might reveal potential bugs. 

2. **Physics Abnormalities**: Try making actors perform unlikely physical behaviors. For example, make a vehicle move at unrealistically high speeds or attempt to climb steep slopes. These sorts of stress tests can help identify potential problems with the physics engine.

3. **Simultaneous and Rapid Sensor Input Changes**: Set up a situation where all sensors on a vehicle are triggered at once (collision sensor, lidar, cameras, etc.), or where the sensor inputs change rapidly. This might help to reveal issues with how CARLA handles simultaneous or rapidly changing sensor inputs.

4. **Aggressive Driving Behaviors**: Design complex maneuvers like sudden lane changes, hard braking, sharp turns at high speed, or even driving off-road. Such behaviors can push the limits of the driving and physics models.

5. **Light and Weather Conditions**: Test the limits of the rendering engine by creating extreme light or weather conditions. Try switching rapidly between day and night, or between calm weather and a storm. 

6. **Traffic Rule Violations**: Make actors violate traffic rules consistently. This includes running red lights, ignoring stop signs, or going the wrong way down a one-way street. This can test both the traffic enforcement and the collision detection systems.
   1. **Bug Found:** Find bugs in Autopilot mode: Ego would run the red light at certain locations [Bug Report: Trigger Autopilot Mode In Certain Location Cause the Ego Run Red Light #6684](https://github.com/carla-simulator/carla/issues/6684)

7. **Continuous Change in Simulation Parameters**: Constantly change simulation parameters such as the time of day, weather conditions, the number and types of actors in the simulation, etc., while the simulation is running. This might reveal issues in how CARLA handles dynamic changes to the simulation environment.

8. **Actor Collision Scenarios**: Try to create a scenario with an enormous amount of collisions between actors at once. This can stress test the collision detection and response system.

9.  **Scripted Complex Multi-Vehicle Maneuvers**: Try creating complex scenarios like a multi-vehicle chase, a traffic jam with many vehicles reacting to one stopped car, or a situation where multiple vehicles must yield to a pedestrian at a crosswalk.

If you do discover bugs during this process, be sure to document them thoroughly. Include the specific circumstances that caused the bug, the expected behavior, the actual behavior, and any error messages you received. It's also helpful if you can provide a script or set of instructions to reproduce the bug. This information will greatly assist the CARLA team in diagnosing and fixing the problem.