#!/usr/bin/env python

# Copyright (c) 2018-2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Follow leading vehicle scenario:

The scenario realizes a common driving behavior, in which the
user-controlled ego vehicle follows a leading car driving down
a given road. At some point the leading car has to slow down and
finally stop. The ego vehicle has to react accordingly to avoid
a collision. The scenario ends either via a timeout, or if the ego
vehicle stopped close enough to the leading vehicle
"""

import random

import py_trees

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      KeepVelocity,
                                                                      StopVehicle,
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToVehicle,
                                                                               InTriggerDistanceToNextIntersection,
                                                                               DriveDistance,
                                                                               StandStill)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_waypoint_in_distance


class FollowLeadingVehicle(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()
        self._first_vehicle_location = 0 #25
        self._first_vehicle_speed = 0 #10
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._other_actor_stop_in_front_intersection = 20
        self._other_actor_transform = None
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicle, self).__init__("FollowVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

            # Example code how to randomize start location
            # distance = random.randint(20, 80)
            # new_location, _ = get_location_in_distance(self.ego_vehicles[0], distance)
            # waypoint = CarlaDataProvider.get_map().get_waypoint(new_location)
            # waypoint.transform.location.z += 39
            # self.other_actors[0].set_transform(waypoint.transform)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        # first_vehicle_transform = carla.Transform(
        #     carla.Location(self._other_actor_transform.location.x,
        #                    self._other_actor_transform.location.y,
        #                    self._other_actor_transform.location.z - 1),
        #     carla.Rotation(yaw=280))
        # first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
        
        for degree in range(0, 360, 1):
            for x_degree in range(-2,2,1):
                for y_degree in range(-1,1,1):
                    for z_degree in range(-1,1,1):
                    
                        first_vehicle_transform = carla.Transform(
                        carla.Location(self._other_actor_transform.location.x + x_degree,
                                    self._other_actor_transform.location.y + y_degree,
                                    self._other_actor_transform.location.z + z_degree),
                        carla.Rotation(yaw=degree))
                        
                        first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
                        if first_vehicle is not None:
                            print("Here is the error degree: ")
                            
                            print("degree: ",degree)
                            print("x_degree: ",x_degree)
                            print("y_degree: ",y_degree)
                            print("z_degree: ",z_degree)
                            
                            print(first_vehicle_transform)
                            first_vehicle.set_simulate_physics(enabled=False)
                            self.other_actors.append(first_vehicle)
                            return
        # exit()
        
        
        # first_vehicle.set_simulate_physics(enabled=False)
        # self.other_actors.append(first_vehicle)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive until reaching
        the next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # to avoid the other actor blocking traffic, it was spawed elsewhere
        # reset its pose to the required one
        start_transform = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)

        # let the other actor drive until next intersection
        # @todo: We should add some feedback mechanism to respond to ego_vehicle behavior
        driving_to_next_intersection = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_vehicle_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToNextIntersection(
            self.other_actors[0], self._other_actor_stop_in_front_intersection))

        # stop vehicle
        stop = StopVehicle(self.other_actors[0], self._other_actor_max_brake)

        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="StandStill", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform)
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(stop)
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        collision_criterion1 = CollisionTest(self.other_actors[0])
        

        criteria.append(collision_criterion)
        criteria.append(collision_criterion1)
        

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()


class FollowLeadingVehicleWithObstacle(BasicScenario):

    """
    This class holds a scenario similar to FollowLeadingVehicle
    but there is an obstacle in front of the leading vehicle

    This is a single ego vehicle scenario
    """

    timeout = 120            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True):
        """
        Setup all relevant parameters and create scenario
        """
        self._map = CarlaDataProvider.get_map()
        self._first_actor_location = 0
        self._second_actor_location = self._first_actor_location + 0 #+41
        self._first_actor_speed = 0
        self._second_actor_speed = 0
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_max_brake = 1.0
        self._first_actor_transform = None
        self._second_actor_transform = None

        super(FollowLeadingVehicleWithObstacle, self).__init__("FollowLeadingVehicleWithObstacle",
                                                        ego_vehicles,
                                                        config,
                                                        world,
                                                        debug_mode,
                                                        criteria_enable=criteria_enable)
        
        

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)
        second_actor_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._second_actor_location)
        
        first_actor_transform = carla.Transform(
            carla.Location(first_actor_waypoint.transform.location.x,
                           first_actor_waypoint.transform.location.y,
                           first_actor_waypoint.transform.location.z + 2),
            first_actor_waypoint.transform.rotation)
        # self._first_actor_transform = carla.Transform(
        #     carla.Location(first_actor_waypoint.transform.location.x,
        #                    first_actor_waypoint.transform.location.y,
        #                    first_actor_waypoint.transform.location.z + 1),
        #     first_actor_waypoint.transform.rotation)
        yaw_1 = second_actor_waypoint.transform.rotation.yaw + 90
        
        second_actor_transform = carla.Transform(
            carla.Location(second_actor_waypoint.transform.location.x,
                           second_actor_waypoint.transform.location.y,
                           second_actor_waypoint.transform.location.z + 1), #-100 will trigger the potential bug
            carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_1,
                           second_actor_waypoint.transform.rotation.roll))
        
        for degree in range(0, 360, 1):
            for x_degree in range(-2,2,1):
                for y_degree in range(-1,1,1):
                    for z_degree in range(-1,1,1):
                    
                        self._first_actor_transform = carla.Transform(
                            carla.Location(first_actor_waypoint.transform.location.x + x_degree,
                                        first_actor_waypoint.transform.location.y + y_degree,
                                        first_actor_waypoint.transform.location.z + z_degree + 1),
                            carla.Rotation(yaw=degree))
                        
                        
                        
                        first_actor = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', self._first_actor_transform)
                        


                        if first_actor is not None:
                            print("Here is the error degree for first actor: ")
                            
                            print("degree: ",degree)
                            print("x_degree: ",x_degree)
                            print("y_degree: ",y_degree)
                            print("z_degree: ",z_degree)
                            
                            print(first_actor)
                            
                            print('-'*30)
                            first_actor.set_simulate_physics(enabled=True)
                            self.other_actors.append(first_actor)
                            
                            for degree2 in range(0, 360, 1):
                                for x_degree2 in range(-2,2,1):
                                    for y_degree2 in range(-1,1,1):
                                        for z_degree2 in range(-1,1,1):
                                            self._second_actor_transform = carla.Transform(
                                                carla.Location(second_actor_waypoint.transform.location.x+ x_degree2,
                                                second_actor_waypoint.transform.location.y+ y_degree2,
                                                second_actor_waypoint.transform.location.z + z_degree2 + 2),
                                                carla.Rotation(yaw=degree))

                                            second_actor = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', self._second_actor_transform)

                                            if second_actor is not None:
                                                print("Here is the error degree for second actor: ")
                                                
                                                print("degree: ",degree2)
                                                print("x_degree: ",x_degree2)
                                                print("y_degree: ",y_degree2)
                                                print("z_degree: ",z_degree2)
                                                
                                                print(second_actor)
                        
                                                second_actor.set_simulate_physics(enabled=True)
                                                self.other_actors.append(second_actor)
                                                return
                        
        # second_actor_transform = carla.Transform(
        #     carla.Location(second_actor_waypoint.transform.location.x,
        #                    second_actor_waypoint.transform.location.y,
        #                    second_actor_waypoint.transform.location.z - 1), #-100 will trigger the potential bug
        #     carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_1,
        #                    second_actor_waypoint.transform.rotation.roll))
        # self._second_actor_transform = carla.Transform(
        #     carla.Location(second_actor_waypoint.transform.location.x,
        #                    second_actor_waypoint.transform.location.y,
        #                    second_actor_waypoint.transform.location.z + 1),
        #     carla.Rotation(second_actor_waypoint.transform.rotation.pitch, yaw_1,
        #                    second_actor_waypoint.transform.rotation.roll))
        
        
        print('-'*10)
        print(first_actor_transform)
        print(second_actor_transform)
        print('-'*10)
        

        # first_actor = CarlaDataProvider.request_new_actor(
        #     'vehicle.nissan.patrol', first_actor_transform)
        # second_actor = CarlaDataProvider.request_new_actor(
        #     'vehicle.diamondback.century', second_actor_transform)

        # first_actor.set_simulate_physics(enabled=False)
        # second_actor.set_simulate_physics(enabled=False)
        # self.other_actors.append(first_actor)
        # self.other_actors.append(second_actor)

    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle" scenario. After
        invoking this scenario, it will wait for the user controlled vehicle to
        enter the start region, then make the other actor to drive towards obstacle.
        Once obstacle clears the road, make the other actor to drive towards the
        next intersection. Finally, the user-controlled vehicle has to be close
        enough to the other actor to end the scenario.
        If this does not happen within 60 seconds, a timeout stops the scenario
        """

        # let the other actor drive until next intersection
        driving_to_next_intersection = py_trees.composites.Parallel(
            "Driving towards Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)

        obstacle_clear_road = py_trees.composites.Parallel("Obstalce clearing road",
                                                           policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        obstacle_clear_road.add_child(DriveDistance(self.other_actors[1], 4))
        obstacle_clear_road.add_child(KeepVelocity(self.other_actors[1], self._second_actor_speed))

        stop_near_intersection = py_trees.composites.Parallel(
            "Waiting for end position near Intersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        stop_near_intersection.add_child(WaypointFollower(self.other_actors[0], 10))
        stop_near_intersection.add_child(InTriggerDistanceToNextIntersection(self.other_actors[0], 20))

        driving_to_next_intersection.add_child(WaypointFollower(self.other_actors[0], self._first_actor_speed))
        driving_to_next_intersection.add_child(InTriggerDistanceToVehicle(self.other_actors[1],
                                                                          self.other_actors[0], 15))

        # end condition
        endcondition = py_trees.composites.Parallel("Waiting for end position",
                                                    policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL)
        endcondition_part1 = InTriggerDistanceToVehicle(self.other_actors[0],
                                                        self.ego_vehicles[0],
                                                        distance=20,
                                                        name="FinalDistance")
        endcondition_part2 = StandStill(self.ego_vehicles[0], name="FinalSpeed", duration=1)
        endcondition.add_child(endcondition_part1)
        endcondition.add_child(endcondition_part2)

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(ActorTransformSetter(self.other_actors[0], self._first_actor_transform))
        sequence.add_child(ActorTransformSetter(self.other_actors[1], self._second_actor_transform))
        sequence.add_child(driving_to_next_intersection)
        sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        sequence.add_child(TimeOut(3))
        sequence.add_child(obstacle_clear_road)
        sequence.add_child(stop_near_intersection)
        sequence.add_child(StopVehicle(self.other_actors[0], self._other_actor_max_brake))
        sequence.add_child(endcondition)
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))

        return sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])
        collision_criterion1 = CollisionTest(self.other_actors[0])
        collision_criterion2 = CollisionTest(self.other_actors[1])
        
        

        criteria.append(collision_criterion)
        criteria.append(collision_criterion1)
        criteria.append(collision_criterion2)
        

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

class PrintDistanceBetweenActors(py_trees.behaviour.Behaviour):
    """
    This class contains a py_tree behavior that outputs the distance between two CARLA actors.
    """

    def __init__(self, actor1, actor2, name="PrintDistance"):
        """
        Setup actor and name
        """
        super(PrintDistanceBetweenActors, self).__init__(name)
        self._actor1 = actor1
        self._actor2 = actor2

    def update(self):
        """
        Calculate and print the distance between the two actors.
        """
        distance = self._actor1.get_location().distance(self._actor2.get_location())
        print(f"Distance between the two actors: {distance} meters")

        return py_trees.common.Status.RUNNING
    



# class FollowLeadingVehicleWithSideVehicle(BasicScenario):

#     """
#     This class holds everything required for a simple "Follow a leading vehicle with side vehicle"
#     scenario involving two vehicles.  (Traffic Scenario 2)

#     This is a single ego vehicle scenario
#     """

#     timeout = 120            # Timeout of scenario in seconds

#     def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
#                  timeout=60):
#         """
#         Setup all relevant parameters and create scenario

#         If randomize is True, the scenario parameters are randomized
#         """

#         self._map = CarlaDataProvider.get_map()
#         self._first_vehicle_location = 25
#         self._second_vehicle_location = -200  # Location of the second vehicle
#         self._first_vehicle_speed = 10
#         self._second_vehicle_speed = 10  # Speed of the second vehicle
#         self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
#         self._other_actor_max_brake = 1.0
#         self._other_actor_stop_in_front_intersection = 20
#         self._other_actor_transform = None
#         self._other_actor_transform_2 = None  # New Actor
#         # Timeout of scenario in seconds
#         self.timeout = timeout

#         super(FollowLeadingVehicleWithSideVehicle, self).__init__("FollowLeadingVehicleWithSideVehicle",
#                                                    ego_vehicles,
#                                                    config,
#                                                    world,
#                                                    debug_mode,
#                                                    criteria_enable=criteria_enable)

#         if randomize:
#             self._ego_other_distance_start = random.randint(4, 8)

#     def _initialize_actors(self, config):
#         """
#         Custom initialization
#         """

#         first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
        
#         # For the second vehicle, get the waypoint on the opposite lane
#         opposite_lane_waypoint = first_vehicle_waypoint.get_left_lane()
        
#         # For the second vehicle, get the waypoint 100m behind on the opposite lane
#         second_vehicle_waypoint, _ = get_waypoint_in_distance(opposite_lane_waypoint, self._second_vehicle_location)

#         self._other_actor_transform = carla.Transform(
#             carla.Location(first_vehicle_waypoint.transform.location.x,
#                            first_vehicle_waypoint.transform.location.y,
#                            first_vehicle_waypoint.transform.location.z + 1),
#             first_vehicle_waypoint.transform.rotation)
#         self._other_actor_transform_2 = carla.Transform(
#             carla.Location(second_vehicle_waypoint.transform.location.x,
#                            second_vehicle_waypoint.transform.location.y,
#                            second_vehicle_waypoint.transform.location.z + 1),
#             second_vehicle_waypoint.transform.rotation)  # New Actor is set 100m behind on the opposite lane

#         first_vehicle_transform = carla.Transform(
#             carla.Location(self._other_actor_transform.location.x,
#                            self._other_actor_transform.location.y,
#                            self._other_actor_transform.location.z - 500),
#             self._other_actor_transform.rotation)
#         first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
#         first_vehicle.set_simulate_physics(enabled=False)
#         self.other_actors.append(first_vehicle)

#         # Adding new actor
#         second_vehicle_transform = carla.Transform(
#             carla.Location(self._other_actor_transform_2.location.x,
#                            self._other_actor_transform_2.location.y,
#                            self._other_actor_transform_2.location.z - 500),
#             self._other_actor_transform_2.rotation)
#         second_vehicle = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', second_vehicle_transform)
#         second_vehicle.set_simulate_physics(enabled=False)
#         self.other_actors.append(second_vehicle)  # Added to the scenario's actor list

#     def _create_behavior(self):
#         """
#         The scenario defined after is a "follow leading vehicle with side vehicle" scenario.
#         """

#         # Reset poses to the required ones
#         start_transform_1 = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
#         start_transform_2 = ActorTransformSetter(self.other_actors[1], self._other_actor_transform_2)  # New actor

#         # Let both actors drive until next intersection
#         driving_to_next_intersection_1 = self.create_driving_sequence(self.other_actors[0], self._first_vehicle_speed)
#         driving_to_next_intersection_2 = self.create_driving_sequence(self.other_actors[1], self._second_vehicle_speed)  # New actor

#         # Build behavior tree
#         sequence = py_trees.composites.Sequence("Sequence Behavior")
#         sequence.add_child(start_transform_1)
#         sequence.add_child(start_transform_2)  # New actor
#         sequence.add_child(driving_to_next_intersection_1)
#         sequence.add_child(driving_to_next_intersection_2)  # New actor
#         sequence.add_child(ActorDestroy(self.other_actors[0]))
#         sequence.add_child(ActorDestroy(self.other_actors[1]))  # New actor

#         # Add the distance printer to the sequence
#         distance_printer = PrintDistanceBetweenActors(self.other_actors[0], self.other_actors[1])
#         sequence.add_child(distance_printer)

#         return sequence

#     def create_driving_sequence(self, actor, speed):
#         driving_sequence = py_trees.composites.Parallel(
#             "DrivingTowardsIntersection",
#             policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
#         driving_sequence.add_child(WaypointFollower(actor, speed))
#         driving_sequence.add_child(InTriggerDistanceToNextIntersection(
#             actor, self._other_actor_stop_in_front_intersection))

#         return driving_sequence

#     def _create_test_criteria(self):
#         """
#         A list of all test criteria will be created that is later used
#         in parallel behavior tree.
#         """
#         criteria = []

#         collision_criterion = CollisionTest(self.ego_vehicles[0])

#         criteria.append(collision_criterion)

#         return criteria

#     def __del__(self):
#         """
#         Remove all actors upon deletion
#         """
#         self.remove_all_actors()


# class FollowLeadingVehicleWithAheadVehicle(BasicScenario):

#     """
#     This class holds everything required for a "Follow a leading vehicle with ahead vehicle"
#     scenario involving three vehicles.

#     This is a single ego vehicle scenario
#     """

#     def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
#                  timeout=60):
#         """
#         Setup all relevant parameters and create scenario

#         If randomize is True, the scenario parameters are randomized
#         """

#         self._map = CarlaDataProvider.get_map()
#         self._first_vehicle_location = 25
#         self._second_vehicle_location = 40  # Location of the second vehicle
#         self._first_vehicle_speed = 10
#         self._second_vehicle_speed = 10  # Speed of the second vehicle
#         self._first_vehicle_deceleration = 0.5  # Deceleration factor of the first vehicle
#         self._second_vehicle_deceleration = 0  # Deceleration factor of the second vehicle
#         self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
#         self._other_actor_max_brake = 1.0
#         self._other_actor_transform = None
#         self._other_actor_transform_2 = None  # New Actor
#         # Timeout of scenario in seconds
#         self.timeout = timeout

#         super(FollowLeadingVehicleWithAheadVehicle, self).__init__("FollowLeadingVehicleWithAheadVehicle",
#                                                    ego_vehicles,
#                                                    config,
#                                                    world,
#                                                    debug_mode,
#                                                    criteria_enable=criteria_enable)

#     def _initialize_actors(self, config):
#         """
#         Custom initialization
#         """

#         first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_vehicle_location)
#         second_vehicle_waypoint, _ = get_waypoint_in_distance(first_vehicle_waypoint, self._second_vehicle_location - self._first_vehicle_location)

#         self._other_actor_transform = carla.Transform(
#             carla.Location(first_vehicle_waypoint.transform.location.x,
#                            first_vehicle_waypoint.transform.location.y,
#                            first_vehicle_waypoint.transform.location.z + 1),
#             first_vehicle_waypoint.transform.rotation)
#         self._other_actor_transform_2 = carla.Transform(
#             carla.Location(second_vehicle_waypoint.transform.location.x,
#                            second_vehicle_waypoint.transform.location.y,
#                            second_vehicle_waypoint.transform.location.z + 1),
#             second_vehicle_waypoint.transform.rotation)  # New Actor is set 100m ahead

#         first_vehicle_transform = carla.Transform(
#             carla.Location(self._other_actor_transform.location.x,
#                            self._other_actor_transform.location.y,
#                            self._other_actor_transform.location.z - 500),
#             self._other_actor_transform.rotation)
#         first_vehicle = CarlaDataProvider.request_new_actor('vehicle.nissan.patrol', first_vehicle_transform)
#         first_vehicle.set_simulate_physics(enabled=False)
#         self.other_actors.append(first_vehicle)

#         # Adding new actor
#         second_vehicle_transform = carla.Transform(
#             carla.Location(self._other_actor_transform_2.location.x,
#                            self._other_actor_transform_2.location.y,
#                            self._other_actor_transform_2.location.z - 500),
#             self._other_actor_transform_2.rotation)
#         second_vehicle = CarlaDataProvider.request_new_actor('vehicle.tesla.model3', second_vehicle_transform)
#         second_vehicle.set_simulate_physics(enabled=False)
#         self.other_actors.append(second_vehicle)  # Added to the scenario's actor list

#     def create_driving_sequence(self, actor, speed, distance, braking_force):
#         """
#         Create a sequence: Drive for a certain distance then stop.
#         """
#         # Drive behavior
#         drive = py_trees.composites.Parallel(
#             policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
#         drive.add_child(WaypointFollower(actor, speed))
#         drive.add_child(DriveDistance(actor, distance))

#         # Stop behavior
#         stop = StopVehicle(actor, braking_force)

#         # Combine drive and stop behaviors into a sequence
#         sequence = py_trees.composites.Sequence()
#         sequence.add_child(drive)
#         sequence.add_child(stop)

#         return sequence


#     def _create_behavior(self):
#         """
#         The scenario defined after is a "Follow a leading vehicle with ahead vehicle" scenario.
#         """

#         # Reset actors' poses to the required ones
#         start_transform_1 = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
#         start_transform_2 = ActorTransformSetter(self.other_actors[1], self._other_actor_transform_2)

#         # Create driving sequences for both actors
#         driving_sequence_1 = self.create_driving_sequence(self.other_actors[0], self._first_vehicle_speed, 100, 0.5)  # lower deceleration for first vehicle
#         driving_sequence_2 = self.create_driving_sequence(self.other_actors[1], self._second_vehicle_speed, 100, 1.0)

#         # Create parallel composite to run the vehicles simultaneously
#         parallel_drive = py_trees.composites.Parallel("Parallel Behavior",
#                                                   policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
#         parallel_drive.add_child(driving_sequence_1)
#         parallel_drive.add_child(driving_sequence_2)

#         # Create end condition to stop the scenario
#         end_condition = StandStill(self.ego_vehicles[0], name="StandStill")

#         # Build behavior tree
#         sequence = py_trees.composites.Sequence("Sequence Behavior")
#         sequence.add_child(start_transform_1)
#         sequence.add_child(start_transform_2)
#         sequence.add_child(parallel_drive)
#         sequence.add_child(end_condition)
#         sequence.add_child(ActorDestroy(self.other_actors[0]))
#         sequence.add_child(ActorDestroy(self.other_actors[1]))

#         return sequence




#     def _create_test_criteria(self):
#         """
#         A list of all test criteria will be created that is later used
#         in parallel behavior tree.
#         """
#         criteria = []

#         # collision_criterion = CollisionTest(self.ego_vehicles[0])
#         collision_criterion = CollisionTest(self.other_actors[0])


#         criteria.append(collision_criterion)

#         return criteria

#     def __del__(self):
#         """
#         Remove all actors upon deletion
#         """
#         self.remove_all_actors()
