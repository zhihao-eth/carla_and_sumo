#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys

import numpy.random as random

from queue import Queue
from queue import Empty

import random
import time


try:
    import numpy as np
except ImportError:
    raise RuntimeError(
        'cannot import numpy, make sure numpy package is installed')

# ==============================================================================
# -- Find CARLA module ---------------------------------------------------------
# ==============================================================================
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

# ==============================================================================
# -- Add PythonAPI for release mode --------------------------------------------
# ==============================================================================
try:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla')
except IndexError:
    pass

import carla
from carla import ColorConverter as cc
from carla import Transform, Location, Rotation, Vector3D

# from behavior_agent import BehaviorAgent
from agents.navigation.behavior_agent import BehaviorAgent  # pylint: disable=import-error
from agents.navigation.basic_agent import BasicAgent  # pylint: disable=import-error


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    if 'lidar' in sensor_name:
        sensor_data.save_to_disk('_out/%06d.png' % sensor_data.frame)
    if 'camera' in sensor_name:
        sensor_data.save_to_disk('_out/%06d.png' % sensor_data.frame)
    sensor_queue.put((sensor_data.frame, sensor_name))


def main():
    actor_list = []

    try:


        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        world = client.load_world('Town06')


        # set sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        # set weather
        weather = carla.WeatherParameters(cloudiness=10.0,
                                          precipitation=0.0,
                                          sun_altitude_angle=70.0,
                                          fog_density=0.0)
        world.set_weather(weather)


        # Traffic manager and its sync mode
        tm = client.get_trafficmanager(8000)
        tm.set_synchronous_mode(True) 

        # The world contains the list blueprints that we can use for adding new
        # actors into the simulation.

        blueprint_library = world.get_blueprint_library()

        spawn_points = world.get_map().get_spawn_points()
        random.shuffle(spawn_points)


        picked_spawn_points = []

        #Spawn an actor at specified point by using Transform
        #lag Vehicle
        spawn_point_v1 = Transform(Location(x=590.297424, y=-20.8, z=0.285300), 
                Rotation(pitch=0, yaw=180, roll=0))

        # Now let's filter all the blueprints of type 'vehicle' and choose one
        # at random.
        vehicle1_bp = world.get_blueprint_library().find('vehicle.dodge.charger_police_2020')

        # A blueprint contains the list of attributes that define a vehicle's
        # instance, we can read them and modify some of them. For instance,
        # let's randomize its color.
        if vehicle1_bp.has_attribute('color'):
            vehicle1_bp.set_attribute('color', '255,255,255')

        # Now we need to give an initial transform to the vehicle. We choose a
        # random transform from the list of recommended spawn points of the map.

        vehicle1_spawn_point = spawn_point_v1 
        vehicle1 = world.spawn_actor(vehicle1_bp, spawn_point_v1 )
        picked_spawn_points.append(spawn_point_v1 )

        velocity1 = Vector3D(x=0, y=0, z=0)

        vehicle1.set_target_velocity(velocity1)

        # So let's tell the world to spawn the vehicle.
        # vehicle1 = world.spawn_actor(vehicle1_bp, transform)

        # It is important to note that the actors we create won't be destroyed
        # unless we call their "destroy" function. If we fail to call "destroy"
        # they will stay in the simulation even after we quit the Python script.
        # For that reason, we are storing all the actors we create so we can
        # destroy them afterwards.
        actor_list.append(vehicle1)
        print('created %s' % vehicle1.type_id)

        NUMBER_OF_VEHICLES = 2

        vehicle_bps = world.get_blueprint_library().filter('vehicle.*.*')

        vehicle_bps = [x for x in vehicle_bps if int(x.get_attribute('number_of_wheels')) == 4]

        vehicle_list = []


        spawn_point_v2 = Transform(Location(x=598.297424, y=-20.8, z=0.285300), 
                Rotation(pitch=0, yaw=180, roll=0))

        spawn_point_v3 = Transform(Location(x=580.297424, y=-20.8, z=0.285300), 
                Rotation(pitch=0, yaw=180, roll=0))


        vehicle_bp = np.random.choice(vehicle_bps)
        try:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point_v2)
            picked_spawn_points.append(spawn_point_v2)
            vehicle_list.append(vehicle)
            vehicle.set_autopilot(True)
            print('created %s' % vehicle.type_id)

        except:
            print('failed')
            pass

        vehicle_bp = np.random.choice(vehicle_bps)
        try:
            vehicle = world.spawn_actor(vehicle_bp, spawn_point_v3)
            picked_spawn_points.append(spawn_point_v3)
            vehicle_list.append(vehicle)
            # vehicle.set_autopilot(True)
            print('created %s' % vehicle.type_id)

        except:
            print('failed')
            pass

        tm = client.get_trafficmanager()
        tm.global_percentage_speed_difference(70.0)

        tm_port = tm.get_port()
        for v in vehicle_list:
            v.set_autopilot(True, tm_port)
            tm.ignore_lights_percentage(v,0)
            tm.distance_to_leading_vehicle(v,0)
            tm.vehicle_percentage_speed_difference(v,70)

        # create sensor queue
        sensor_queue = Queue(maxsize=10)

        # Let's add now a camera attached to the vehicle. Note that the
        # transform we give here is now relative to the vehicle.
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_transform = carla.Transform(carla.Location(x=-5, z=2))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle1)
        actor_list.append(camera)
        print('created %s' % camera.type_id)

        # Now we register the function that will be called each time the sensor
        # receives an image. In this example we are saving the image to disk

        camera.listen(lambda image: sensor_callback(image, sensor_queue, "camera"))


        # we need to tick the world once to let the client update the spawn position
        world.tick()

        # create the behavior agent
        agent = BehaviorAgent(vehicle1, behavior='normal')

        # set the destination spot
        destination = Transform(Location(x=-207.209152, y=247.122726, z=0.300000), 
                Rotation(pitch=0, yaw=0, roll=0))

        print('moved vehicle from %s' % agent._vehicle.get_location())    
        print('moved vehicle to %s' % destination.location)

        # generate the route
        agent.set_destination(destination.location)


        while True:
            agent._update_information()

            world.tick()
            data = sensor_queue.get(block=True)

            if len(agent._local_planner._waypoints_queue)<1:
                print('======== Success, Arrivied at Target Point!')
                break

            # top view
            spectator = world.get_spectator()
            transform = vehicle1.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=40),
                                                    carla.Rotation(pitch=-90)))

            speed_limit = vehicle1.get_speed_limit()
            agent.get_local_planner().set_speed(speed_limit)

            control = agent.run_step(debug=True)
            vehicle1.apply_control(control)



    finally:

        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)

        print('destroying actors')
        camera.destroy()
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicle_list])
        print('done.')


if __name__ == '__main__':

    main()
    
