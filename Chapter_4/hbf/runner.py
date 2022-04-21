#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2009-2022 German Aerospace Center (DLR) and others.
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# https://www.eclipse.org/legal/epl-2.0/
# This Source Code may also be made available under the following Secondary
# Licenses when the conditions for such availability set forth in the Eclipse
# Public License 2.0 are satisfied: GNU General Public License, version 2
# or later which is available at
# https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

# @file    runner.py
# @author  Lena Kalleske
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @author  Jakob Erdmann
# @date    2009-03-26

from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import random
import math

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa
import sumolib

def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 3600  # number of time steps
    # demand per second from different directions
    pWE = 1. / 10
    pEW = 1. / 11
    pNS = 1. / 30
    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" \
guiShape="passenger"/>
        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="down" edges="54o 4i 3o 53i" />""", file=routes)
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('    <vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pEW:
                print('    <vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
            if random.uniform(0, 1) < pNS:
                print('    <vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
        print("</routes>", file=routes)

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>


def fix_angle_range(angle):
    '''
    return the angle between (-180°, 180°]
    '''    
    while(angle <= 180):
        angle += 2 * 180;
    
    angle -= 2 * 180 * int(angle / (2*180))    
    return angle > 180 and (angle - 2 * 180) or angle 


def get_angel_2_vehicles(ego_vehicle, ref_vehicle):
    '''
    returns the angle relative to the front of ego_vehicle and the ref_vehicle
    '''
    position_ego_vehicle = traci.vehicle.getPosition(ego_vehicle)
    position_ref_vehicle = traci.vehicle.getPosition(ref_vehicle)
    
    distance_x = float(position_ref_vehicle[0] - position_ego_vehicle[0])
    distance_y = float(position_ref_vehicle[1] - position_ego_vehicle[1])
    
    angle_x_y = 90 - math.degrees(math.atan2(distance_y , distance_x))
    
    angle_ego_vehicle = fix_angle_range(traci.vehicle.getAngle(ego_vehicle))
    
    included_angle = fix_angle_range(angle_x_y - angle_ego_vehicle)
    
    direction = 'straight'
    if included_angle > 0:
        direction = 'right'
    elif included_angle < 0:
        direction = 'left'
        
    ref_angle = [direction, abs(included_angle)]   
    return ref_angle 


def get_distance_2_vehicles(vehicle_1,vehicle_2):
    '''
    returns the distance between two vehicles
    '''
    position_1 = traci.vehicle.getPosition(vehicle_1)
    position_2 = traci.vehicle.getPosition(vehicle_2)
    
    distance_x = position_2[0] - position_1[0]
    distance_y = position_2[1] - position_1[1]
    
    distance_2d = math.sqrt( (distance_x**2) + (distance_y**2) )
    return distance_2d

def print_leader(ego_vehicle):
    '''
    print information for the leader
    '''
    current_leader = traci.vehicle.getLeader(ego_vehicle)
    if current_leader != None:
        leader = current_leader[0]
        distance_leader = current_leader[1]
        angle_leader = ['straight']
        print('↑ ','vehicle_id:',leader,'  distance:',distance_leader,'  direction:',angle_leader)  

def print_left_leaders(ego_vehicle):
    '''
    print information for all left leader vehicles
    '''
    current_left_leaders = traci.vehicle.getNeighbors(ego_vehicle,2)
    if current_left_leaders != []:
        for i in range(len(current_left_leaders)):
            left_leaders = current_left_leaders[i][0]
            distance_left_leaders = get_distance_2_vehicles(ego_vehicle, left_leaders)
            # distance_left_leaders = current_left_leaders[i][1]
            angle_left_leaders = get_angel_2_vehicles(ego_vehicle, left_leaders)
            print('↖ ','vehicle_id:',left_leaders,'  distance:',distance_left_leaders,'  direction:',angle_left_leaders)   

def print_left_followers(ego_vehicle):
    '''
    print information for all left follower vehicles
    '''
    current_left_followers = traci.vehicle.getNeighbors(ego_vehicle,0)
    if current_left_followers != []:
        for i in range(len(current_left_followers)):
            left_followers = current_left_followers[i][0]
            distance_left_followers = get_distance_2_vehicles(ego_vehicle, left_followers)
            # distance_left_followers = current_left_followers[i][1]
            angle_left_followers = get_angel_2_vehicles(ego_vehicle, left_followers)
            print('↙ ','vehicle_id:',left_followers,'  distance:',distance_left_followers,'  direction:',angle_left_followers) 
        
def print_right_leaders(ego_vehicle):
    '''
    print information for all right leader vehicles
    '''
    current_right_leaders = traci.vehicle.getNeighbors(ego_vehicle,3)
    if current_right_leaders != []:
        for i in range(len(current_right_leaders)):
            right_leaders = current_right_leaders[i][0]
            distance_right_leaders = get_distance_2_vehicles(ego_vehicle, right_leaders)
            # distance_right_leaders = current_right_leaders[i][1]
            angle_right_leaders = get_angel_2_vehicles(ego_vehicle, right_leaders)
            print('↗ ','vehicle_id:',right_leaders,'  distance:',distance_right_leaders,'  direction:',angle_right_leaders)  
    
def print_right_followers(ego_vehicle):
    '''
    print information for all right follower vehicles
    '''
    current_right_followers = traci.vehicle.getNeighbors(ego_vehicle,1)
    if current_right_followers != []:
        for i in range(len(current_right_followers)):
            right_followers = current_right_followers[i][0]
            distance_right_followers = get_distance_2_vehicles(ego_vehicle, right_followers)
            # distance_right_followers = current_right_followers[i][1]
            angle_right_followers = get_angel_2_vehicles(ego_vehicle, right_followers)
            print('↘ ','vehicle_id:',right_followers,'  distance:',distance_right_followers,'  direction:',angle_right_followers) 

def print_all_neighbors(ego_vehicle):
    '''
    print information for all neighbor vehicles
    '''
    print('schow neighboring vehicles: ')
    print_leader(ego_vehicle)
    print_left_leaders(ego_vehicle)
    print_left_followers(ego_vehicle)
    print_right_leaders(ego_vehicle)
    print_right_followers(ego_vehicle)

def run():
    """execute the TraCI control loop"""
    step = 0
    
    # read networl information through sumolib
    sumo_net = sumolib.net.readNet('hbf.net.xml')
    
    # set 'vehicle_0' as ego vehicle
    ego_vehicle = 'vehicle_0'
    
    # start looping in steps until all vehicles have completed their path
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        
        print(step)
        
        try:
            
            # print the current x,y-coordinates of the vehicle
            current_x_y_position = traci.vehicle.getPosition(ego_vehicle)
            print('x_y_position: ',current_x_y_position)

            # print the current GPS-coordinates of the vehicle
            x_0 = current_x_y_position[0]
            y_0 = current_x_y_position[1]
            current_gps_position = sumo_net.convertXY2LonLat(float(x_0), float(y_0))
            print('gps_position: ',current_gps_position)
            
            # print the current angle of the vehicle's orientation in Cartesian coordinates
            current_angle = traci.vehicle.getAngle(ego_vehicle)
            current_angle = fix_angle_range(90 - current_angle)
            print('ego_angle: ',current_angle)
            
            # print the current speed of the vehicle
            current_speed = traci.vehicle.getSpeed(ego_vehicle)
            print('ego_speed: ',current_speed)          
            
            # print information for all neighbor vehicles of ego vehicle 
            print_all_neighbors(ego_vehicle)

        except:
            
            # if ego vehicle is not in the network
            print('ego vehicle is not in the network')
        
        step += 1
    traci.close()
    sys.stdout.flush()

def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # first, generate the route file for this simulation
    # generate_routefile()

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "hbf.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
