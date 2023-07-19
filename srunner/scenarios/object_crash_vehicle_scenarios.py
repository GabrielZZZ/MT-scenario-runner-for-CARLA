#!/usr/bin/env python
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Object crash without prior vehicle action scenario:
The scenario realizes the user controlled ego vehicle
moving along the road and encountering a cyclist ahead.
"""

from __future__ import print_function

import math
import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (ActorTransformSetter,
                                                                      ActorDestroy,
                                                                      AccelerateToVelocity,
                                                                      HandBrakeVehicle,
                                                                      KeepVelocity,
                                                                      StopVehicle)
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import (InTriggerDistanceToLocationAlongRoute,
                                                                               InTimeToArrivalToVehicle,
                                                                               DriveDistance)
from srunner.scenariomanager.timer import TimeOut
from srunner.scenarios.basic_scenario import BasicScenario
from srunner.tools.scenario_helper import get_location_in_distance_from_wp



class DynamicObjectCrossingTwoPedstrians(BasicScenario):

    """
    This class holds everything required for a simple object crash
    without prior vehicle action involving a vehicle and a cyclist/pedestrian,
    The ego vehicle is passing through a road,
    And encounters a cyclist/pedestrian crossing the road.

    This is a single ego vehicle scenario
    """

    def __init__(self, world, ego_vehicles, config, randomize=False,
                 debug_mode=False, criteria_enable=True, adversary_type=False, timeout=20):
        """
        Setup all relevant parameters and create scenario
        """
        self._wmap = CarlaDataProvider.get_map()

        # Store the new parameters
        # print("first_actor_location: ", config.first_actor_location)
        # print("second_actor_location: ", config.second_actor_location)
        # print("first_actor_speed: ", config.first_actor_speed)
        # print("other_actor_max_brake: ", config.other_actor_max_brake)
        # print("other_actor_stop_in_front_intersection: ", config.other_actor_stop_in_front_intersection)
        # print("first_actor_type: ", config.first_actor_type)
        # print("second_actor_type: ", config.second_actor_type)

        self._start_distance = config.first_actor_location # the distance between the first pedestrain and the ego vehicle
        self._second_actor_location = config.second_actor_location # the distance between the second pedestrain and the first pedestrain
        self._second_actor_speed = config.second_actor_speed
        self._other_actor_max_brake = config.other_actor_max_brake
        # self._other_actor_stop_in_front_intersection = config.other_actor_stop_in_front_intersection
        # self._second_actor_type = config.second_actor_type
        # self._first_actor_type = config.first_actor_type

        self._reference_waypoint = self._wmap.get_waypoint(config.trigger_points[0].location)
        # ego vehicle parameters
        self._ego_vehicle_distance_driven = 100 # the distance of the ego vehicle to drive after passing the actors
        # other vehicle parameters
        self._other_actor_target_velocity = 100 # original: 5
        self._other_actor_max_brake = 1.0
        self._time_to_reach = 30
        self._adversary_type = adversary_type  # flag to select either pedestrian (False) or cyclist (True)
        self._walker_yaw = 0
        self._walker_yaw1 = 0

        self._num_lane_changes = 1
        self._other_actor_transform = None

        # Add a third pedestrian
        self._other_actor_transform_2 = None

        self._other_actor_transform_3 = None # this for blocker

        self.timeout = timeout
        self._trigger_location = config.trigger_points[0].location
        # Total Number of attempts to relocate a vehicle before spawning
        self._number_of_attempts = 20
        # Number of attempts made so far
        self._spawn_attempted = 0

        self._ego_route = CarlaDataProvider.get_ego_vehicle_route()

        super(DynamicObjectCrossingTwoPedstrians, self).__init__("DynamicObjectCrossingTwoPedstrians",
                                                    ego_vehicles,
                                                    config,
                                                    world,
                                                    debug_mode,
                                                    criteria_enable=criteria_enable)

    def _calculate_base_transform(self, _start_distance, waypoint):

        lane_width = waypoint.lane_width

        # Patches false junctions
        if self._reference_waypoint.is_junction:
            stop_at_junction = False
        else:
            stop_at_junction = True

        location, _ = get_location_in_distance_from_wp(waypoint, _start_distance, stop_at_junction)
        waypoint = self._wmap.get_waypoint(location)
        offset = {"orientation": 270, "position": 90, "z": 0.6, "k": 1.0}
        position_yaw = waypoint.transform.rotation.yaw + offset['position']
        orientation_yaw = waypoint.transform.rotation.yaw + offset['orientation']
        offset_location = carla.Location(
            offset['k'] * lane_width * math.cos(math.radians(position_yaw)),
            offset['k'] * lane_width * math.sin(math.radians(position_yaw)))
        location += offset_location
        location.z = self._trigger_location.z + offset['z']
        return carla.Transform(location, carla.Rotation(yaw=orientation_yaw)), orientation_yaw

    def _spawn_adversary(self, transform, orientation_yaw, base_speed):

        self._time_to_reach *= self._num_lane_changes

        if self._adversary_type is False:
            self._walker_yaw1 = orientation_yaw
            self._other_actor_target_velocity = base_speed + (0.4 * self._num_lane_changes)
            walker = CarlaDataProvider.request_new_actor('walker.*', transform)

            # self._walker_yaw2 = orientation_yaw
            # # self._other_actor_target_velocity = base_speed + (0.1 * self._num_lane_changes)
            # walker2 = CarlaDataProvider.request_new_actor('walker.*', transform)

            adversary = walker
        else:
            self._other_actor_target_velocity = self._other_actor_target_velocity * self._num_lane_changes
            first_vehicle = CarlaDataProvider.request_new_actor('vehicle.diamondback.century', transform)
            first_vehicle.set_simulate_physics(enabled=False)
            adversary = first_vehicle

        return adversary

    def _spawn_blocker(self, transform, orientation_yaw, shift):
        """
        Spawn the blocker prop that blocks the vision from the egovehicle of the jaywalker
        :return:
        """
        # static object transform
        # shift = 0.9
        x_ego = self._reference_waypoint.transform.location.x
        y_ego = self._reference_waypoint.transform.location.y
        x_cycle = transform.location.x
        y_cycle = transform.location.y
        x_static = x_ego + shift * (x_cycle - x_ego)
        y_static = y_ego + shift * (y_cycle - y_ego)

        spawn_point_wp = self.ego_vehicles[0].get_world().get_map().get_waypoint(transform.location)

        self._other_actor_transform_3 = carla.Transform(carla.Location(x_static, y_static,
                                                         spawn_point_wp.transform.location.z + 0.3),
                                          carla.Rotation(yaw=orientation_yaw + 180))
        
 
        static = CarlaDataProvider.request_new_actor('static.prop.vendingmachine', self._other_actor_transform_3)
        static.set_simulate_physics(enabled=False)

        return static

    def _initialize_actors(self, config):
        """
        Custom initialization
        """
        # cyclist transform
        inital_start_distance = self._start_distance

        # print("initial_start_distance: ", inital_start_distance)


        vehicles = []
        # We start by getting and waypoint in the closest sidewalk.
        waypoint = self._reference_waypoint
        while True:
            wp_next = waypoint.get_right_lane()

            # wp_next = waypoint.get_left_lane()

            self._num_lane_changes += 1
            # print("num_lane_changes: ", self._num_lane_changes)
            if wp_next is None or wp_next.lane_type == carla.LaneType.Sidewalk:
                

                break
            elif wp_next.lane_type == carla.LaneType.Shoulder:
                # Filter Parkings considered as Shoulders
                if wp_next.lane_width > 2:
                    inital_start_distance += 1.5
                    waypoint = wp_next
                print(waypoint.transform.location.x)
                print(wp_next.transform.location.x)
                print(waypoint.transform.location.y)
                print(wp_next.transform.location.y)

                break
            else:
                inital_start_distance += 1.5
                waypoint = wp_next
    

        print("initial_start_distance: ", inital_start_distance)
        while True:  # We keep trying to spawn avoiding props

            try:
                self._other_actor_transform, orientation_yaw = self._calculate_base_transform(inital_start_distance, waypoint)
                # first_vehicle = self._spawn_adversary(self._other_actor_transform, orientation_yaw, 4)
                self._other_actor_transform_2, orientation_yaw = self._calculate_base_transform(inital_start_distance+self._second_actor_location, waypoint)

                vehicles.append(self._spawn_adversary(self._other_actor_transform, orientation_yaw, 7))
                vehicles.append(self._spawn_adversary(self._other_actor_transform_2, orientation_yaw, 5))



                # blocker1 = self._spawn_blocker(self._other_actor_transform3_, orientation_yaw, 0.9)

                # self._other_actor_transform, orientation_yaw = self._calculate_base_transform(inital_start_distance+2, waypoint)

                # second_vehicle = self._spawn_adversary(self._other_actor_transform, orientation_yaw, 4)
                # blocker2 = self._spawn_blocker(self._other_actor_transform, orientation_yaw, 2.0)


                break
            except RuntimeError as r:
                # We keep retrying until we spawn
                print("Base transform is blocking objects ", self._other_actor_transform)
                inital_start_distance += 0.4
                self._spawn_attempted += 1
                if self._spawn_attempted >= self._number_of_attempts:
                    raise r

        # Now that we found a possible position we just put the vehicle to the underground
        disp_transform1 = carla.Transform(
            carla.Location(self._other_actor_transform.location.x,
                           self._other_actor_transform.location.y,
                           self._other_actor_transform.location.z - 500),
            self._other_actor_transform.rotation)

        # prop_disp_transform = carla.Transform(
        #     carla.Location(self._other_actor_transform_3.location.x,
        #                    self._other_actor_transform_3.location.y,
        #                    self._other_actor_transform_3.location.z - 500),
        #     self._other_actor_transform_3.rotation)

        vehicles[0].set_transform(disp_transform1)

        disp_transform2 = carla.Transform(
            carla.Location(self._other_actor_transform_2.location.x,
                           self._other_actor_transform_2.location.y,
                           self._other_actor_transform_2.location.z - 500),
            self._other_actor_transform_2.rotation)

        vehicles[1].set_transform(disp_transform2)

        # blocker1.set_transform(prop_disp_transform)

        # blocker2.set_transform(prop_disp_transform)


        vehicles[0].set_simulate_physics(enabled=False)

        vehicles[1].set_simulate_physics(enabled=False)


        # blocker1.set_simulate_physics(enabled=False)

        # blocker2.set_simulate_physics(enabled=False)

        self.other_actors.append(vehicles[0])

        self.other_actors.append(vehicles[1])


        # self.other_actors.append(blocker1)

        # self.other_actors.append(blocker2)


    def _create_behavior(self):
        """
        After invoking this scenario, cyclist will wait for the user
        controlled vehicle to enter trigger distance region,
        the cyclist starts crossing the road once the condition meets,
        then after 60 seconds, a timeout stops the scenario
        """

        root = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="OccludedObjectCrossing")
        lane_width = self._reference_waypoint.lane_width
        lane_width = lane_width + (1.25 * lane_width * self._num_lane_changes)

        dist_to_trigger = 13 + self._num_lane_changes
        # leaf nodes
        if self._ego_route is not None:
            start_condition1 = InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                    self._ego_route,
                                                                    self._other_actor_transform.location,
                                                                    dist_to_trigger)
            
            start_condition2 = InTriggerDistanceToLocationAlongRoute(self.ego_vehicles[0],
                                                                    self._ego_route,
                                                                    self._other_actor_transform.location+2,
                                                                    dist_to_trigger)
        else:
            start_condition1 = InTimeToArrivalToVehicle(self.ego_vehicles[0],
                                                       self.other_actors[0],
                                                       self._time_to_reach)
            
            start_condition2 = InTimeToArrivalToVehicle(self.ego_vehicles[0],
                                                       self.other_actors[1],
                                                       self._time_to_reach+2)

        actor_velocity1 = KeepVelocity(self.other_actors[0],
                                      self._other_actor_target_velocity,
                                      name="walker velocity")
        actor_drive1 = DriveDistance(self.other_actors[0],
                                    0.5 * lane_width,
                                    name="walker drive distance")
        actor_start_cross_lane1 = AccelerateToVelocity(self.other_actors[0],
                                                      1.0,
                                                      self._other_actor_target_velocity,
                                                      name="walker crossing lane accelerate velocity")
        actor_cross_lane1 = DriveDistance(self.other_actors[0],
                                         lane_width+10,
                                         name="walker drive distance for lane crossing ")
        actor_stop_crossed_lane1 = StopVehicle(self.other_actors[0],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        

        actor_velocity2 = KeepVelocity(self.other_actors[1],
                                      self._second_actor_speed,
                                      name="walker velocity")
        actor_drive2 = DriveDistance(self.other_actors[1],
                                    0.5 * lane_width,
                                    name="walker drive distance")
        actor_start_cross_lane2 = AccelerateToVelocity(self.other_actors[1],
                                                      1.0,
                                                      self._other_actor_target_velocity,
                                                      name="walker crossing lane accelerate velocity")
        actor_cross_lane2 = DriveDistance(self.other_actors[1],
                                         lane_width,
                                         name="walker drive distance for lane crossing ")
        actor_stop_crossed_lane2 = StopVehicle(self.other_actors[1],
                                              self._other_actor_max_brake,
                                              name="walker stop")
        

        ego_pass_machine = DriveDistance(self.ego_vehicles[0],
                                         5,
                                         name="ego vehicle passed prop")
        actor_remove1 = ActorDestroy(self.other_actors[0],
                                    name="Destroying walker1")
        actor_remove2 = ActorDestroy(self.other_actors[1],
                                    name="Destroying walker2")
        

        # static_remove1 = ActorDestroy(self.other_actors[2],
        #                              name="Destroying Prop")
        
        # static_remove2 = ActorDestroy(self.other_actors[3],
        #                              name="Destroying Prop")
        
        end_condition = DriveDistance(self.ego_vehicles[0],
                                      self._ego_vehicle_distance_driven,
                                      name="End condition ego drive distance")

        # non leaf nodes

        scenario_sequence = py_trees.composites.Sequence()
        keep_velocity_other1 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
        keep_velocity1 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")
        
        keep_velocity_other2 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity other")
        keep_velocity2 = py_trees.composites.Parallel(
            policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="keep velocity")

        # building tree

        root.add_child(scenario_sequence)
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[0], self._other_actor_transform,
                                                         name='TransformSetterTS3walker1'))
        # scenario_sequence.add_child(ActorTransformSetter(self.other_actors[2], self._other_actor_transform_2,
        #                                                  name='TransformSetterTS3coca1', physics=False))
        
        scenario_sequence.add_child(ActorTransformSetter(self.other_actors[1], self._other_actor_transform_2,
                                                         name='TransformSetterTS3walker2'))
        # scenario_sequence.add_child(ActorTransformSetter(self.other_actors[3], self._other_actor_transform2,
        #                                                  name='TransformSetterTS3coca2', physics=False))

        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], True))
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[1], True))

        scenario_sequence.add_child(start_condition1)
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[0], False))
        scenario_sequence.add_child(HandBrakeVehicle(self.other_actors[1], False))

        scenario_sequence.add_child(keep_velocity1)
        scenario_sequence.add_child(keep_velocity2)

        scenario_sequence.add_child(keep_velocity_other1)
        scenario_sequence.add_child(keep_velocity_other2)

        scenario_sequence.add_child(actor_stop_crossed_lane1)
        scenario_sequence.add_child(actor_stop_crossed_lane2)

        scenario_sequence.add_child(actor_remove1)
        scenario_sequence.add_child(actor_remove2)

        # scenario_sequence.add_child(static_remove1)
        scenario_sequence.add_child(end_condition)


        keep_velocity1.add_child(actor_velocity1)
        keep_velocity1.add_child(actor_drive1)
        keep_velocity_other1.add_child(actor_start_cross_lane1)
        keep_velocity_other1.add_child(actor_cross_lane1)
        keep_velocity_other1.add_child(ego_pass_machine)

        keep_velocity2.add_child(actor_velocity2)
        keep_velocity2.add_child(actor_drive2)
        keep_velocity_other2.add_child(actor_start_cross_lane2)
        keep_velocity_other2.add_child(actor_cross_lane2)
        keep_velocity_other2.add_child(ego_pass_machine)

        return root

    def _create_test_criteria(self):
        """
        A list of all test criteria will be created that is later used
        in parallel behavior tree.
        """
        criteria = []

        collision_criterion1 = CollisionTest(self.ego_vehicles[0])
        collision_criterion2 = CollisionTest(self.other_actors[0])
        collision_criterion3 = CollisionTest(self.other_actors[1])

        criteria.append(collision_criterion1)
        criteria.append(collision_criterion2)
        criteria.append(collision_criterion3)

        return criteria

    def __del__(self):
        """
        Remove all actors upon deletion
        """
        self.remove_all_actors()
