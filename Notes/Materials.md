# Test function: Spawn Actor

[carla.Actor API](../../../../../tmp/mume202361-13063-1hrig6v.9755.html)

Autopilot mode will subscribe a vehicle to the Traffic Manager to simulate real urban conditions. This module is hard-coded, not based on machine learning.

**Can we focused on the specific actor first and use script to monitor the properties?**
Such as we spaw an pedestrian, and we set its location, then we monitor the actor behavior. We can spawn multiple actors and let them interact with each other either directly or indirectly, then we construct MRs.

## Task 1
1. **Create one actor, set its properties, and monitor its behaviors with codes**
2. **Use ChatGPT to generate source scenarios**


### Source scenarios from ChatGPT
These MRs may be incorrect, but the scenarios can be used as source scenarios.

**Distance Keeping:**
MR evaluation criteria: 
The distance between actors + the speed of actors

**MR1:** If we increase the speed of all actors in the simulation proportionally, the distance maintained between vehicles should remain the same.

**MR2:** If we duplicate the size of all actors in the simulation, the distance maintained between the actors should increase proportionally to the increase in size.

**Overtaking Behaviour:**

**MR1:** If we double the speed of a following car, it should still be able to overtake the leading car without collision.

**MR2:** If we reverse the order of cars (i.e., make the leading car the following car and vice versa) but keep their speeds constant, the overtaking behavior should not happen as the former following car (now leading) was originally faster.

**Pedestrian Crossing:**
***Scenario Template:** The exisiting source test scenario: carla-simulator/scenario_runner-0.9.13/srunner/scenarios/object_crash_vehicle.py*

**MR5:** If we add more pedestrians intending to cross the street at pedestrian crossings, the number of cars stopping at these pedestrian crossings should increase accordingly.

**MR6:** If we remove pedestrian crossings from the simulation, cars should not stop in these areas.

--------------------------------------------------------------------------------
# List of Supported Scenarios

Welcome to the ScenarioRunner for CARLA! This document provides a list of all
currently supported scenarios, and a short description for each one.

### FollowLeadingVehicle
The scenario realizes a common driving behavior, in which the user-controlled
ego vehicle follows a leading car driving down a given road in Town01. At some
point the leading car slows down and finally stops. The ego vehicle has to react
accordingly to avoid a collision. The scenario ends either via a timeout, or if
the ego vehicle stopped close enough to the leading vehicle

### FollowLeadingVehicleWithObstacle
This scenario is very similar to 'FollowLeadingVehicle'. The only difference is,
that in front of the leading vehicle is a (hidden) obstacle that blocks the way.

### VehicleTurningRight
In this scenario the ego vehicle takes a right turn from an intersection where
a cyclist suddenly drives into the way of the ego vehicle,which has to stop
accordingly. After some time, the cyclist clears the road, such that ego vehicle
can continue driving.

### VehicleTurningLeft
This scenario is similar to 'VehicleTurningRight'. The difference is that the ego
vehicle takes a left turn from an intersection.

### OppositeVehicleRunningRedLight
In this scenario an illegal behavior at an intersection is tested. An other
vehicle waits at an intersection, but illegally runs a red traffic light. The
approaching ego vehicle has to handle this situation correctly, i.e. despite of
a green traffic light, it has to stop and wait until the intersection is clear
again. Afterwards, it should continue driving.

### StationaryObjectCrossing
In this scenario a cyclist is stationary waiting in the middle of the road and
blocking the way for the ego vehicle. Hence, the ego vehicle has to stop in
front of the cyclist.

### DynamicObjectCrossing
This is similar to 'StationaryObjectCrossing', but with the difference that the
cyclist is dynamic. It suddenly drives into the way of the ego vehicle, which
has to stop accordingly. After some time, the cyclist will clear the road, such
that the ego vehicle can continue driving.

### NoSignalJunctionCrossing
This scenario tests negotiation between two vehicles crossing cross each other
through a junction without signal.
The ego vehicle is passing through a junction without traffic lights
And encounters another vehicle passing across the junction. The ego vehicle has
to avoid collision and navigate across the junction to succeed.

### ControlLoss
In this scenario control loss of a vehicle is tested due to bad road conditions, etc
and it checks whether the vehicle is regained its control and corrected its course.

### ManeuverOppositeDirection
In this scenario vehicle is passing another vehicle in a rural area, in daylight, under clear
weather conditions, at a non-junction and encroaches into another
vehicle traveling in the opposite direction.

### OtherLeadingVehicle
The scenario realizes a common driving behavior, in which the user-controlled ego
vehicle follows a leading car driving down a given road.
At some point the leading car has to decelerate. The ego vehicle has to react
accordingly by changing lane to avoid a collision and follow the leading car in
other lane. The scenario ends via timeout, or if the ego vehicle drives certain
distance.

### SignalizedJunctionRightTurn
In this scenario right turn of hero actor without collision at signalized intersection
is tested. Hero Vehicle is turning right in an urban area, at a signalized intersection and
turns into the same direction of another vehicle crossing straight initially from
a lateral direction.

### SignalizedJunctionLeftTurn
In this scenario hero vehicle is turning left in an urban area,
at a signalized intersection and cuts across the path of another vehicle
coming straight crossing from an opposite direction.
