# Metamorphic Testing in Driving Simulators

Metamorphic Testing is a unique approach to testing software that leverages the relations between inputs and outputs across multiple executions, termed as Metamorphic Relations (MRs). This document aims to outline MRs that can be applied in the context of testing driving simulators.

## 1. Spawning Actors
We can monitor the outputs by the collision results, and location data after spawned.

**The code does not allow to spawn two actors at the same or very close location to avoid collision.**

By running the iteration loop of trying to spawn the actor near the ego vehicle, I have identified a problem when try to spawn actors on the ego vehicle where should not be allowed to spawn.

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

- **MR1**: Changing the environment (e.g., from dry to rainy) should cause the vehicle's behavior to change accordingly.
- **MR2**: The vehicle's behavior should revert back to its original state when the environment is changed back to the original state.
- **MR3**: If the environment changes from one state to another and back again, the behavior of the vehicle should revert back to the original state.
- **MR4**: Changing the environment while the vehicle is stationary should not affect the vehicle's position.

## 3. Control Movements

- **MR1**: If the same control movement is input twice in succession, the resulting direction of movement or position should be more to the left compared to just one input (if the control was to steer left, for example).
- **MR2**: If a control movement and then the opposite control movement are performed, the resulting direction of movement or position should be close to the original position before the movements.
- **MR3**: If no control movements are input, the vehicle should continue in its current state.
- **MR4**: If the same control movement is input after a delay, the vehicle's state should change as if the delay did not happen.

## 4. Vehicle Dynamics

- **MR1**: If the same force is applied twice to a vehicle from rest, the resulting speed should be approximately double compared to applying the force once.
- **MR2**: If a force and then the opposite force are applied to a moving vehicle, the resulting speed should be close to the original speed before the forces were applied.
- **MR3**: If the same force is applied to a moving vehicle twice in succession, the resulting speed decrease should be approximately double compared to applying the force once.
- **MR4**: If a vehicle in motion has a force applied until it stops, applying the same force again should not change the vehicle's state.

This document serves as a guide to creating MRs for testing a driving simulator, but it is by no means exhaustive. Additional MRs may be developed based on the specific behavior and characteristics of the simulator being tested.
