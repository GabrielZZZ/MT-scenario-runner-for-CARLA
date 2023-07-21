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


import json

class FollowLeadingVehicleWithAheadVehicle(BasicScenario):

    """
    This class holds everything required for a "Follow a leading vehicle with ahead vehicle"
    scenario involving three vehicles.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=20):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()

        # # Read parameters from configuration file
        # with open("scenario_config.json", "r") as file:
        #     config1 = json.load(file)

        # self._first_actor_location = config1["first_actor_location"]
        # self._second_actor_location = config1["second_actor_location"]
        # self._first_actor_speed = config1["first_actor_speed"]
        # self._other_actor_max_brake = config1["other_actor_max_brake"]

        # Store the new parameters
        print("first_actor_location: ", config.first_actor_location)
        print("second_actor_location: ", config.second_actor_location)
        print("first_actor_speed: ", config.first_actor_speed)
        print("other_actor_max_brake: ", config.other_actor_max_brake)
        print("other_actor_stop_in_front_intersection: ", config.other_actor_stop_in_front_intersection)
        print("first_actor_type: ", config.first_actor_type)
        print("second_actor_type: ", config.second_actor_type)

        self._first_actor_location = config.first_actor_location
        self._second_actor_location = config.second_actor_location
        self._first_actor_speed = config.first_actor_speed
        self._other_actor_max_brake = config.other_actor_max_brake
        self._other_actor_stop_in_front_intersection = config.other_actor_stop_in_front_intersection
        self._second_actor_type = config.second_actor_type
        self._first_actor_type = config.first_actor_type

        # self._first_actor_location = 25
        # self._second_actor_location = 40  # Location of the second vehicle
        # self._first_actor_speed = 10
        self._second_actor_speed = config.second_actor_speed  # Speed of the second vehicle
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        # self._other_actor_max_brake = 1.0
        self._other_actor_transform = None
        self._other_actor_transform_2 = None  # New Actor
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicleWithAheadVehicle, self).__init__("FollowLeadingVehicleWithAheadVehicle",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

    def _initialize_actors(self, config):
        """
        Custom initialization
        """

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)
        second_vehicle_waypoint, _ = get_waypoint_in_distance(first_vehicle_waypoint, self._second_actor_location - self._first_actor_location)

        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            carla.Rotation(yaw=180))
        self._other_actor_transform_2 = carla.Transform(
            carla.Location(second_vehicle_waypoint.transform.location.x,
                           second_vehicle_waypoint.transform.location.y,
                           second_vehicle_waypoint.transform.location.z + 1),
            carla.Rotation(yaw=180))  # New Actor is set 100m ahead

# (180,40) would fail, but (180,41) would success


        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        
        print('-'*10)
        print(first_vehicle_transform)
        
        
        first_vehicle = CarlaDataProvider.request_new_actor(self._first_actor_type, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

        # Adding new actor
        second_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform_2.location.x,
                           self._other_actor_transform_2.location.y,
                           self._other_actor_transform_2.location.z - 500),
            self._other_actor_transform_2.rotation)
        
        print(second_vehicle_transform)
        print('-'*10)
        
        
        second_vehicle = CarlaDataProvider.request_new_actor(self._second_actor_type, second_vehicle_transform)

        # Existing code...      
        second_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle)  # Added to the scenario's actor list

    def create_driving_sequence(self, actor, speed, distance, braking_force):
        """
        Create a sequence: Drive for a certain distance then stop.
        """
        # Drive behavior
        drive = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        drive.add_child(WaypointFollower(actor, speed))
        drive.add_child(DriveDistance(actor, distance))

        # Stop behavior
        stop = StopVehicle(actor, braking_force)

        # Combine drive and stop behaviors into a sequence
        sequence = py_trees.composites.Sequence()
        sequence.add_child(drive)
        sequence.add_child(stop)

        return sequence


    def _create_behavior(self):
        """
        The scenario defined after is a "Follow a leading vehicle with ahead vehicle" scenario.
        """

        # Reset actors' poses to the required ones
        start_transform_1 = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
        start_transform_2 = ActorTransformSetter(self.other_actors[1], self._other_actor_transform_2)

        # Create driving sequences for both actors
        driving_sequence_1 = self.create_driving_sequence(self.other_actors[0], self._first_actor_speed, 200, 0.5)  # lower deceleration for first vehicle
        driving_sequence_2 = self.create_driving_sequence(self.other_actors[1], self._second_actor_speed, 200, 1.0)

        # Create parallel composite to run the vehicles simultaneously
        parallel_drive = py_trees.composites.Parallel("Parallel Behavior",
                                                  policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        parallel_drive.add_child(driving_sequence_1)
        parallel_drive.add_child(driving_sequence_2)

        # Create end condition to stop the scenario
        end_condition = StandStill(self.ego_vehicles[0], name="StandStill")

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform_1)
        sequence.add_child(start_transform_2)
        sequence.add_child(parallel_drive)
        sequence.add_child(end_condition)
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


class FollowLeadingVehicleWithLeftSideVehicle(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle with side vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 60            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()

        # Read parameters from configuration file
        
        # Store the new parameters
        self._first_actor_location = config.first_actor_location
        self._second_actor_location = config.second_actor_location
        self._first_actor_speed = config.first_actor_speed
        self._other_actor_max_brake = config.other_actor_max_brake
        self._other_actor_stop_in_front_intersection = config.other_actor_stop_in_front_intersection
        self._second_actor_type = config.second_actor_type
        self._first_actor_type = config.first_actor_type

        
        self._second_actor_speed = config.second_actor_speed  # Speed of the second vehicle
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_transform = None
        self._other_actor_transform_2 = None  # New Actor
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicleWithLeftSideVehicle, self).__init__("FollowLeadingVehicleWithLeftSideVehicle",
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

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)

        # For the second vehicle, get the waypoint on the opposite lane
        opposite_lane_waypoint = first_vehicle_waypoint.get_left_lane()
        # opposite_lane_waypoint = first_vehicle_waypoint.get_right_lane()


        # For the second vehicle, get the waypoint 100m behind on the opposite lane
        second_vehicle_waypoint, _ = get_waypoint_in_distance(opposite_lane_waypoint, self._second_actor_location)

        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        self._other_actor_transform_2 = carla.Transform(
            carla.Location(second_vehicle_waypoint.transform.location.x,
                           second_vehicle_waypoint.transform.location.y,
                           second_vehicle_waypoint.transform.location.z + 1),
            second_vehicle_waypoint.transform.rotation)  # New Actor is set 100m behind on the opposite lane

        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        # print("first_actor_type: ", self._first_actor_type)
        # print("second_actor_type: ", self._second_actor_type)
        first_vehicle = CarlaDataProvider.request_new_actor(self._first_actor_type, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

        # Adding new actor
        second_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform_2.location.x,
                           self._other_actor_transform_2.location.y,
                           self._other_actor_transform_2.location.z - 500),
            self._other_actor_transform_2.rotation)
        second_vehicle = CarlaDataProvider.request_new_actor(self._second_actor_type, second_vehicle_transform)
        second_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle)  # Added to the scenario's actor list


    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle with side vehicle" scenario.
        """

        # Reset poses to the required ones
        start_transform_1 = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
        start_transform_2 = ActorTransformSetter(self.other_actors[1], self._other_actor_transform_2)  # New actor

        # Let both actors drive until next intersection
        driving_to_next_intersection_1 = self.create_driving_sequence(self.other_actors[0], self._first_actor_speed)
        driving_to_next_intersection_2 = self.create_driving_sequence(self.other_actors[1], self._second_actor_speed)  # New actor

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform_1)
        sequence.add_child(start_transform_2)  # New actor
        sequence.add_child(driving_to_next_intersection_1)
        sequence.add_child(driving_to_next_intersection_2)  # New actor
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))  # New actor

        return sequence

    def create_driving_sequence(self, actor, speed):
        driving_sequence = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_sequence.add_child(WaypointFollower(actor, speed))
        driving_sequence.add_child(InTriggerDistanceToNextIntersection(
            actor, self._other_actor_stop_in_front_intersection))

        return driving_sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

class FollowLeadingVehicleWithRightSideVehicle(BasicScenario):

    """
    This class holds everything required for a simple "Follow a leading vehicle with side vehicle"
    scenario involving two vehicles.  (Traffic Scenario 2)

    This is a single ego vehicle scenario
    """

    timeout = 60            # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        """
        Setup all relevant parameters and create scenario

        If randomize is True, the scenario parameters are randomized
        """

        self._map = CarlaDataProvider.get_map()

        # Read parameters from configuration file
        
        # Store the new parameters
        self._first_actor_location = config.first_actor_location
        self._second_actor_location = config.second_actor_location
        self._first_actor_speed = config.first_actor_speed
        self._other_actor_max_brake = config.other_actor_max_brake
        self._other_actor_stop_in_front_intersection = config.other_actor_stop_in_front_intersection
        self._second_actor_type = config.second_actor_type
        self._first_actor_type = config.first_actor_type

        
        self._second_actor_speed = config.second_actor_speed  # Speed of the second vehicle
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_transform = None
        self._other_actor_transform_2 = None  # New Actor
        # Timeout of scenario in seconds
        self.timeout = timeout

        super(FollowLeadingVehicleWithRightSideVehicle, self).__init__("FollowLeadingVehicleWithRightSideVehicle",
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

        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)

        # For the second vehicle, get the waypoint on the opposite lane
        # opposite_lane_waypoint = first_vehicle_waypoint.get_left_lane()
        opposite_lane_waypoint = first_vehicle_waypoint.get_right_lane()


        # For the second vehicle, get the waypoint 100m behind on the opposite lane
        second_vehicle_waypoint, _ = get_waypoint_in_distance(opposite_lane_waypoint, self._second_actor_location)

        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        self._other_actor_transform_2 = carla.Transform(
            carla.Location(second_vehicle_waypoint.transform.location.x,
                           second_vehicle_waypoint.transform.location.y,
                           second_vehicle_waypoint.transform.location.z + 1),
            second_vehicle_waypoint.transform.rotation)  # New Actor is set 100m behind on the opposite lane

        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        # print("first_actor_type: ", self._first_actor_type)
        # print("second_actor_type: ", self._second_actor_type)
        first_vehicle = CarlaDataProvider.request_new_actor(self._first_actor_type, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

        # Adding new actor
        second_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform_2.location.x,
                           self._other_actor_transform_2.location.y,
                           self._other_actor_transform_2.location.z - 500),
            self._other_actor_transform_2.rotation)
        second_vehicle = CarlaDataProvider.request_new_actor(self._second_actor_type, second_vehicle_transform)
        second_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle)  # Added to the scenario's actor list


    def _create_behavior(self):
        """
        The scenario defined after is a "follow leading vehicle with side vehicle" scenario.
        """

        # Reset poses to the required ones
        start_transform_1 = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
        start_transform_2 = ActorTransformSetter(self.other_actors[1], self._other_actor_transform_2)  # New actor

        # Let both actors drive until next intersection
        driving_to_next_intersection_1 = self.create_driving_sequence(self.other_actors[0], self._first_actor_speed)
        driving_to_next_intersection_2 = self.create_driving_sequence(self.other_actors[1], self._second_actor_speed)  # New actor

        # Build behavior tree
        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform_1)
        sequence.add_child(start_transform_2)  # New actor
        sequence.add_child(driving_to_next_intersection_1)
        sequence.add_child(driving_to_next_intersection_2)  # New actor
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))  # New actor

        return sequence

    def create_driving_sequence(self, actor, speed):
        driving_sequence = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_sequence.add_child(WaypointFollower(actor, speed))
        driving_sequence.add_child(InTriggerDistanceToNextIntersection(
            actor, self._other_actor_stop_in_front_intersection))

        return driving_sequence

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()

class FollowLeadingVehicleWithSideVehicleCombined(BasicScenario):
    timeout = 60  # Timeout of scenario in seconds

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        self._map = CarlaDataProvider.get_map()

        # Read parameters from configuration file
        self._first_actor_location = config.first_actor_location
        self._second_actor_location = config.second_actor_location
        self._first_actor_speed = config.first_actor_speed
        self._other_actor_max_brake = config.other_actor_max_brake
        self._other_actor_stop_in_front_intersection = config.other_actor_stop_in_front_intersection
        self._second_actor_type = config.second_actor_type
        self._first_actor_type = config.first_actor_type

        self._second_actor_speed = config.second_actor_speed  # Speed of the second vehicle
        self._reference_waypoint = self._map.get_waypoint(config.trigger_points[0].location)
        self._other_actor_transform = None
        self._other_actor_transform_left = None  # New Actor for left lane
        self._other_actor_transform_right = None  # New Actor for right lane
        self.timeout = timeout

        super(FollowLeadingVehicleWithSideVehicleCombined, self).__init__("FollowLeadingVehicleWithSideVehicleCombined",
                                                   ego_vehicles,
                                                   config,
                                                   world,
                                                   debug_mode,
                                                   criteria_enable=criteria_enable)

        if randomize:
            self._ego_other_distance_start = random.randint(4, 8)

    def _initialize_actors(self, config):
        first_vehicle_waypoint, _ = get_waypoint_in_distance(self._reference_waypoint, self._first_actor_location)

        # For the second vehicle, get the waypoint on the opposite lane
        opposite_lane_waypoint_left = first_vehicle_waypoint.get_left_lane()
        opposite_lane_waypoint_right = first_vehicle_waypoint.get_right_lane()

        # For the second vehicle, get the waypoint 100m behind on the opposite lane
        second_vehicle_waypoint_left, _ = get_waypoint_in_distance(opposite_lane_waypoint_left, self._second_actor_location)
        second_vehicle_waypoint_right, _ = get_waypoint_in_distance(opposite_lane_waypoint_right, self._second_actor_location)

        self._other_actor_transform = carla.Transform(
            carla.Location(first_vehicle_waypoint.transform.location.x,
                           first_vehicle_waypoint.transform.location.y,
                           first_vehicle_waypoint.transform.location.z + 1),
            first_vehicle_waypoint.transform.rotation)
        self._other_actor_transform_left = carla.Transform(
            carla.Location(second_vehicle_waypoint_left.transform.location.x,
                           second_vehicle_waypoint_left.transform.location.y,
                           second_vehicle_waypoint_left.transform.location.z + 1),
            second_vehicle_waypoint_left.transform.rotation)  # New Actor is set 100m behind on the left lane
        self._other_actor_transform_right = carla.Transform(
            carla.Location(second_vehicle_waypoint_right.transform.location.x,
                           second_vehicle_waypoint_right.transform.location.y,
                           second_vehicle_waypoint_right.transform.location.z + 1),
            second_vehicle_waypoint_right.transform.rotation)  # New Actor is set 100m behind on the right lane

        first_vehicle_transform = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)
        first_vehicle = CarlaDataProvider.request_new_actor(self._first_actor_type, first_vehicle_transform)
        first_vehicle.set_simulate_physics(enabled=False)
        self.other_actors.append(first_vehicle)

        # Adding new actor for left lane
        second_vehicle_transform_left = carla.Transform(
            carla.Location(self._other_actor_transform_left.location.x,
                           self._other_actor_transform_left.location.y,
                           self._other_actor_transform_left.location.z - 500),
            self._other_actor_transform_left.rotation)
        second_vehicle_left = CarlaDataProvider.request_new_actor(self._second_actor_type, second_vehicle_transform_left)
        second_vehicle_left.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle_left)  # Added to the scenario's actor list

        # Adding new actor for right lane
        second_vehicle_transform_right = carla.Transform(
            carla.Location(self._other_actor_transform_right.location.x,
                           self._other_actor_transform_right.location.y,
                           self._other_actor_transform_right.location.z - 500),
            self._other_actor_transform_right.rotation)
        second_vehicle_right = CarlaDataProvider.request_new_actor(self._second_actor_type, second_vehicle_transform_right)
        second_vehicle_right.set_simulate_physics(enabled=False)
        self.other_actors.append(second_vehicle_right)  # Added to the scenario's actor list


    def _create_behavior(self):
        start_transform_1 = ActorTransformSetter(self.other_actors[0], self._other_actor_transform)
        start_transform_2_left = ActorTransformSetter(self.other_actors[1], self._other_actor_transform_left)  # New actor for left lane
        start_transform_2_right = ActorTransformSetter(self.other_actors[2], self._other_actor_transform_right)  # New actor for right lane

        driving_to_next_intersection_1 = self.create_driving_sequence(self.other_actors[0], self._first_actor_speed)
        driving_to_next_intersection_2_left = self.create_driving_sequence(self.other_actors[1], self._second_actor_speed)  # New actor for left lane
        driving_to_next_intersection_2_right = self.create_driving_sequence(self.other_actors[2], self._second_actor_speed)  # New actor for right lane

        sequence = py_trees.composites.Sequence("Sequence Behavior")
        sequence.add_child(start_transform_1)
        sequence.add_child(start_transform_2_left)  # New actor for left lane
        sequence.add_child(start_transform_2_right)  # New actor for right lane
        sequence.add_child(driving_to_next_intersection_1)
        sequence.add_child(driving_to_next_intersection_2_left)  # New actor for left lane
        sequence.add_child(driving_to_next_intersection_2_right)  # New actor for right lane
        sequence.add_child(ActorDestroy(self.other_actors[0]))
        sequence.add_child(ActorDestroy(self.other_actors[1]))  # New actor for left lane
        sequence.add_child(ActorDestroy(self.other_actors[2]))  # New actor for right lane

        return sequence

    def create_driving_sequence(self, actor, speed):
        driving_sequence = py_trees.composites.Parallel(
            "DrivingTowardsIntersection",
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE)
        driving_sequence.add_child(WaypointFollower(actor, speed))
        driving_sequence.add_child(InTriggerDistanceToNextIntersection(
            actor, self._other_actor_stop_in_front_intersection))

        return driving_sequence

    def _create_test_criteria(self):
        criteria = []

        collision_criterion = CollisionTest(self.ego_vehicles[0])

        criteria.append(collision_criterion)

        return criteria

    def __del__(self):
        self.remove_all_actors()
