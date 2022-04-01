# My traffic routing algorithm
# Written by Mike Yannuzzi
# from abc import ABC, abstractmethod
import random
import os
import sys
from core.Util import *
from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy
import csv
from controller.algoHelper import getDeadline

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")
import sumolib

STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"


# My class for routing - inherits from RouteController
class MikeGorithm(RouteController):
    """
    Example class for a custom scheduling algorithm.
    Utilizes a random decision policy until vehicle destination is within reach,
    then targets the vehicle destination.
    """
    def __init__(self, connection_info):
        super().__init__(connection_info)

    def make_decisions(self, vehicles, connection_info):
        """
        A custom scheduling algorithm can be written in between the 'Your algo...' comments.
        -For each car in the vehicle batch, your algorithm should provide a list of future decisions.
        -Sometimes short paths result in the vehicle reaching its local TRACI destination before reaching its
         true global destination. In order to counteract this, ask for a list of decisions rather than just one.
        -This list of decisions is sent to a function that returns the 'closest viable target' edge
          reachable by the decisions - it is not the case that all decisions will always be consumed.
          As soon as there is enough distance between the current edge and the target edge, the compute_target_edge
          function will return.
        -The 'closest viable edge' is a local target that is used by TRACI to control vehicles
        -The closest viable edge should always be far enough away to ensure that the vehicle is not removed
          from the simulation by TRACI before the vehicle reaches its true destination

        :param vehicles: list of vehicles to make routing decisions for
        :param connection_info: object containing network information
        :return: local_targets: {vehicle_id, target_edge}, where target_edge is a local target to send to TRACI
        """

        """
        Algorithm steps: (Runs for every turn)
        1. Sort Vehicles by longest deadline
            -Gives vehicles with longer deadlines chance to choose move first
        2. Calculate the travel time for each vehicle based on end point, then calculate the mean travel time of all cars
            -Get travel time from shortest path length (using Djikstra's) then calculate the mean travel for all cars
        3. Calculate the congestion ratio for each potential direction
            -The congest ratio is the ratio of cars to an edge (# of cars/edge length)
        4. Pick between the two:
            -If the mean travel time isn't lowered enough from the potential choices then choose the direction with the lowest congestion ratio
        """
        decision_list = []
        local_targets = {}
        print("Using Mike's algorithm!")
        print(str(len(vehicles)))
        print("1. Sorting vehicles")
        # Creating and sorting list outside of function
        # Vehicle ID list
        # for v in vehicles:
        #     print(v.vehicle_id)
            # agent = [v.vehicle_id, v.deadline]
            # print("VEHIClE ID = " + str(agent[0]) + " DEADLINE = " + str(agent[1]))
            # vId_list.append(agent)
            # print("VEHIClE ID = " + str(v.vehicle_id) + " DEADLINE = " + str(v.deadline))
        # print(vId_list)
        # Sort list by deadlines
        # vSorted = vId_list.sort(key=self.getDeadline(vId_list))
        # for i in vSorted:
        #     print(str(vSorted[1]))
        # Print the sorted list
        vSorted = sorted(vehicles, key= lambda d: d.deadline, reverse=True)
        # for i in range(len(vSorted)):
            # print(vSorted[i])
        # Calculate mean deadline
        meanDeadline = -1
        sumDeadline = 0
        for i in vSorted:
            sumDeadline = sumDeadline + vSorted[i].deadline
        # meanDeadline = sumDeadline/float()
        print("MEAN DEADLINE IS: " + str(len(vSorted)))
        for vehicle in vSorted:
            start_edge = vehicle.current_edge
            print("IN THE LOOP")
            print("Working on vehicle: " + str(vehicle.vehicle_id))
            '''
            Your algo starts here
            '''
            
            # Create sorted list of car id's then reference the cars by id when making decisions
            # First create a copy of the list of car id's
            
            # # vehicle_ids = set(vehicles.getIDList())
            # for v in vehicles:
            #     # print(vehicle.vehicle_id)
            #     agent = [v.vehicle_id, v.deadline]
            #     # print("VEHIClE ID = " + str(agent[0]) + " DEADLINE = " + str(agent[1]))
            #     vId_list.append(agent)
            #     # print("VEHIClE ID = " + str(v.vehicle_id) + " DEADLINE = " + str(v.deadline))
            
            # Putting this here for now - Can't test without this
            choice = self.direction_choices[random.randint(0, 5)]
            decision_list.append(choice)
            '''
            Your algo ends here
            '''
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        # print(vId_list)
        return local_targets

    