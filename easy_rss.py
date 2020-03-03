#!/bin/env python

# ----------------- BEGIN LICENSE BLOCK ---------------------------------
#
# Copyright (c) 2019-2020 Intel Corporation
#
# SPDX-License-Identifier: LGPL-2.1-only
#
# ----------------- END LICENSE BLOCK -----------------------------------

"""
Simple unittest module to ensure that the Python binding is functional
"""
import unittest

import libad_rss_python as rss
import libad_physics_python as physics


class InterfaceTest(unittest.TestCase):

    """
    Test class for Python interface
    """

    world_model = None

    road_length = 1000.0
    road_width = 5.0

    def _prepare_occupied_region(self, segment_id):
        occupied_region = rss.OccupiedRegion()
        occupied_region.segmentId = segment_id
        return occupied_region




    def _prepare_vehicle(self, vehicle_type, object_id, car_width, car_length, road_width, road_length, car_rear_end_lon, car_right_lat, car_speed, occupied_region):
        """
        Setup Ego vehicle
        """
        rss_vehicle_a = rss.Object()

        rss_vehicle_a.objectType = vehicle_type
        rss_vehicle_a.velocity.speedLonMin = physics.Speed(car_speed)
        rss_vehicle_a.velocity.speedLonMax = physics.Speed(car_speed)
        rss_vehicle_a.velocity.speedLatMin = physics.Speed(0.0)
        rss_vehicle_a.velocity.speedLatMax = physics.Speed(0.0)
        rss_vehicle_a.objectId = object_id

        occupied_region.latRange.minimum = physics.ParametricValue(car_right_lat)
        occupied_region.latRange.maximum = physics.ParametricValue((car_right_lat + car_width) / road_width)
        occupied_region.lonRange.minimum = physics.ParametricValue(car_rear_end_lon/ road_length)
        occupied_region.lonRange.maximum = physics.ParametricValue((car_rear_end_lon+ car_length) / road_length)

        rss_vehicle_a.occupiedRegions.append(occupied_region)

        return rss_vehicle_a

    def _prepare_scene_same_direction(self, road_width, road_length, ego_rear_end_lon, other_rear_end_lon):

        occupied_region = self._prepare_occupied_region(0)
        ego_vehicle = self._prepare_vehicle(rss.ObjectType.EgoVehicle, 0, 2.0, 5.0, road_width, road_length, ego_rear_end_lon, 0.0, 20.0, occupied_region)
        other_vehicle = self._prepare_vehicle(rss.ObjectType.OtherVehicle, 1, 2.0, 5.0, road_width, road_length, other_rear_end_lon, 0.0, 10.0, occupied_region)

        rss_scene = rss.Scene()

        rss_scene.egoVehicle = ego_vehicle
        rss_scene.object = other_vehicle
        rss_scene.situationType = rss.SituationType.SameDirection

        road_segment = rss.RoadSegment()
        lane_segment = rss.LaneSegment()
        lane_segment.id = 0
        lane_segment.length.minimum = physics.Distance(road_length)
        lane_segment.length.maximum = physics.Distance(road_length)
        lane_segment.width.minimum = physics.Distance(road_width)
        lane_segment.width.maximum = physics.Distance(road_width)
        lane_segment.type = rss.LaneSegmentType.Normal
        lane_segment.drivingDirection = rss.LaneDrivingDirection.Positive
        road_segment.append(lane_segment)
        rss_scene.egoVehicleRoad.append(road_segment)
        rss_scene.objectRssDynamics.responseTime = physics.Duration(0.5)
        rss_scene.objectRssDynamics.alphaLat.accelMax = physics.Acceleration(1.)
        rss_scene.objectRssDynamics.alphaLat.brakeMin = physics.Acceleration(1.)
        rss_scene.objectRssDynamics.alphaLon.accelMax = physics.Acceleration(4.)
        rss_scene.objectRssDynamics.alphaLon.brakeMax = physics.Acceleration(8.)
        rss_scene.objectRssDynamics.alphaLon.brakeMinCorrect = physics.Acceleration(4.)
        rss_scene.objectRssDynamics.alphaLon.brakeMin = physics.Acceleration(4.)
        rss_scene.objectRssDynamics.lateralFluctuationMargin = physics.Distance(0.)

        return rss_scene

    def _prepare_world_model2(self, rss_scene, time_index):
        self.world_model = rss.WorldModel()

        self.world_model.timeIndex = time_index
        self.world_model.scenes.append(rss_scene)
        self.world_model.egoVehicleRssDynamics.responseTime = physics.Duration(0.2)
        self.world_model.egoVehicleRssDynamics.alphaLat.accelMax = physics.Acceleration(0.1)
        self.world_model.egoVehicleRssDynamics.alphaLat.brakeMin = physics.Acceleration(0.1)
        self.world_model.egoVehicleRssDynamics.alphaLon.accelMax = physics.Acceleration(0.1)
        self.world_model.egoVehicleRssDynamics.alphaLon.brakeMax = physics.Acceleration(8.)
        self.world_model.egoVehicleRssDynamics.alphaLon.brakeMin = physics.Acceleration(4.)
        self.world_model.egoVehicleRssDynamics.alphaLon.brakeMinCorrect = physics.Acceleration(4.)
        self.world_model.egoVehicleRssDynamics.lateralFluctuationMargin = physics.Distance(0.)


    def _check_RSS_complaince(self, world_model):
        print ("== Input World Model ==")
        print(world_model)

        rss_response_resolving = rss.RssResponseResolving()
        rss_situation_checking = rss.RssSituationChecking()
        rss_sitation_extraction = rss.RssSituationExtraction()

        rss_situation_snapshot = rss.SituationSnapshot()
        self.assertTrue(rss_sitation_extraction.extractSituations(world_model, rss_situation_snapshot))

        print ("== Situation Snaphost ==")
        print (rss_situation_snapshot)

        rss_state_snapshot = rss.RssStateSnapshot()
        self.assertTrue(rss_situation_checking.checkSituations(rss_situation_snapshot, rss_state_snapshot))

        rss_proper_response = rss.ProperResponse()
        self.assertTrue(rss_response_resolving.provideProperResponse(rss_state_snapshot, rss_proper_response))

        print ("== Proper Response ==")
        print (rss_proper_response)

        acceleration_restriction = rss.AccelerationRestriction()
        self.assertTrue(rss.transformProperResponse(world_model, rss_proper_response, acceleration_restriction))

        print ("== Acceleration Restrictions ==")
        print (acceleration_restriction)

        longitudinal_distance = float(rss_situation_snapshot.situations[0].relativePosition.longitudinalDistance)
        print ("---->", longitudinal_distance)
        self.assertTrue(rss_proper_response.isSafe)
        #self.assertEqual(longitudinal_distance, 95.0)



    def test_interface(self):
        """
        Main test part
        """

        #self._prepare_world_model()

        for i in range(100):
            rss_scene = self._prepare_scene_same_direction(5.0, 1000.0, 0.0+20.0*i, 100.0+10.0*i)
            self._prepare_world_model2(rss_scene, i+1)
            self._check_RSS_complaince(self.world_model)




if __name__ == '__main__':
    unittest.main()
