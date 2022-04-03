# My traffic routing algorithm
# Written by Mike Yannuzzi
# from abc import ABC, abstractmethod
import random
import os
import sys
from threading import current_thread
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
        self.meanDeadline = []
        self.simSteps = []
        self.updatedMean = []

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

        '''
        Your algo starts here
        '''
        # Starting over because theres no speed value
        # Sort the vehicles by largest deadline
        # print(str(len(vehicles)))
        vSorted = sorted(vehicles, key= lambda d: d.deadline, reverse=True)
        currentMeanDeadline = 0
        sumDeadline = 0
        currentCount = traci.vehicle.getIDCount()
        # Make sure count is at least 1
        if currentCount < 1:
            currentCount =1
        # print("CURRENT VEHICLE COUNT: " + str(currentCount))
        for vehicle in vSorted:
            # currentCount += 1
            # print("VEHICLE DEADLINE IS: " + str(vehicle.deadline))
            sumDeadline = sumDeadline + vehicle.deadline
            # print("SUM OF DEADLINE: " + str(sumDeadline))
        
        currentMeanDeadline = sumDeadline/currentCount
        # print("MEAN DEADLINE IS: " + str(currentMeanDeadline))
        # Append the current mean deadline and the sim time to the arrays
        self.meanDeadline.append(currentMeanDeadline)
        self.simSteps.append(traci.simulation.getTime())
        for vehicle in vSorted:
            maxSpeed = traci.vehicle.getMaxSpeed(vehicle.vehicle_id)
            # print("MAX SPEED = " + str(maxSpeed))
            #print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
            decision_list = []
            unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
            visited = {} # map of visited edges
            current_edge = vehicle.current_edge

            current_distance = self.connection_info.edge_length_dict[current_edge]
            unvisited[current_edge] = current_distance
            path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
            while True:
                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge not in unvisited:
                        continue
                    # Get the congestion ratio of the current edge
                    # number of cars/edge length
                    edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                    carNum = self.connection_info.edge_vehicle_count[outgoing_edge]
                    # print("CAR NUM IS: " + str(carNum))
                    # Calculate congestion ratio
                    congestionRatio = carNum/edge_length
                    # print("CONGESTION RATIO: " + str(congestionRatio))
                    # Add the congestion ratio to the current edge length
                    new_distance = current_distance + edge_length + congestionRatio
                    # add the new_distance/vehicle speed to new mean deadline
                    newDeadline = new_distance/maxSpeed
                    # copy the list
                    vCopy = vSorted.copy()
                    # save the current vehicle id to change its value
                    idCopy = vehicle.vehicle_id
                    # Replace the current vehicle id with the new deadline and calc a new mean
                    for vehicle in vCopy:
                        if vehicle.vehicle_id == idCopy:
                            vehicle.deadline = newDeadline
                    #vCopy.vehicle.deadline = newDeadline
                    for vehicle in vCopy:
                        sumDeadline = sumDeadline + vehicle.deadline
                        # print("SUM OF DEADLINE: " + str(sumDeadline))
                    currentCount = traci.vehicle.getIDCount()
                    newMeanDeadline = sumDeadline/currentCount
                    # Save the new mean
                    self.updatedMean.append(newMeanDeadline)
                    # Change the herusitic here for choosing next edge
                    if new_distance < unvisited[outgoing_edge]:
                        unvisited[outgoing_edge] = new_distance
                        current_path = copy.deepcopy(path_lists[current_edge])
                        current_path.append(direction)
                        path_lists[outgoing_edge] = copy.deepcopy(current_path)
                        #print("{} + {} : {} + {}".format(path_lists[current_edge], direction, path_edge_lists[current_edge], outgoing_edge))

                visited[current_edge] = current_distance
                del unvisited[current_edge]
                if not unvisited:
                    break
                if current_edge==vehicle.destination:
                    break
                possible_edges = [edge for edge in unvisited.items() if edge[1]]
                current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]
                #print('{}:{}------------'.format(current_edge, current_distance))
            #current_edge = vehicle.current_edge




        # print("Using Mike's algorithm!")
        # Print the vehicles list out
        # for v in vehicles:
        #     print("id: " + str(v.vehicle_id) + " destination: " + str(v.destination) + " start time: " + str(v.start_time) + " deadline: " + str(v.deadline)) 
            # print("id: {}, destination: {}, start time:{}, deadline: {};".format(vid, \
            # v.destination, v.start_time, v.deadline))
        
        # print("1. Sorting vehicles")
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
        # vSorted = sorted(vehicles, key= lambda d: d.deadline, reverse=True)
        # for i in range(len(vSorted)):
            # print(vSorted[i])
        # Calculate mean deadline
        # meanDeadline = -1
        # sumDeadline = 0
        # for i in vSorted:
        #     sumDeadline = sumDeadline + vSorted[i].deadline
        # # meanDeadline = sumDeadline/float()
        # print("MEAN DEADLINE IS: " + str(len(vSorted)))
        # start_edge = vehicle.current_edge
        # print("IN THE LOOP")
        # print("Working on vehicle: " + str(vehicle.vehicle_id))
        # print("id: " + str(vehicle.vehicle_id) + " destination: " + str(vehicle.destination) + " start time: " + str(vehicle.start_time) + " deadline: " + str(vehicle.deadline)) 
        
        # print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
        # Grab
        # meanDeadline = 0
        # for vehicle in vehicles:
        #     meanDeadline = meanDeadline+vehicle.deadline
        #     print("MEAN DEADLINE = " + str(meanDeadline))
        #     # print("Sort list of vehicles by longest deadline...")
        #     vSorted = sorted(vehicles, key= lambda d: d.deadline, reverse=True)
        #     # for vehicles in vSorted:
        #     #     print(str(vehicle.deadline))
        #     # Now that the vehicles are sorted grab the potential next turns
        #     # First get the current edge
        #     current_edge = vehicle.current_edge
        #     # Grab all the outgoing edges from that key edge
        #     # print(str(self.connection_info.outgoing_edges_dict[current_edge]))
        #     # print(str(self.connection_info.outgoing_edges_dict))
        #     # print(str(current_edge))
        #     crList =[]
        # for vehicle in vSorted:
        #     # Perform djikstra's
        #     #print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
        #     decision_list = []
        #     unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
        #     visited = {} # map of visited edges
        #     current_edge = vehicle.current_edge

        #     current_distance = self.connection_info.edge_length_dict[current_edge]
        #     unvisited[current_edge] = current_distance
        #     path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
        #     while True:
        #         if current_edge not in self.connection_info.outgoing_edges_dict.keys():
        #             continue
        #         for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
        #             if outgoing_edge not in unvisited:
        #                 continue
        #             edge_length = self.connection_info.edge_length_dict[outgoing_edge]
        #             new_distance = current_distance + edge_length
        #             if new_distance < unvisited[outgoing_edge]:
        #                 unvisited[outgoing_edge] = new_distance
        #                 current_path = copy.deepcopy(path_lists[current_edge])
        #                 current_path.append(direction)
        #                 path_lists[outgoing_edge] = copy.deepcopy(current_path)
        #                 #print("{} + {} : {} + {}".format(path_lists[current_edge], direction, path_edge_lists[current_edge], outgoing_edge))

        #         visited[current_edge] = current_distance
        #         del unvisited[current_edge]
        #         if not unvisited:
        #             break
        #         if current_edge==vehicle.destination:
        #             break
        #         possible_edges = [edge for edge in unvisited.items() if edge[1]]
        #         current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]

        #         # Grab the congestion ratio
        #         # Create a list of the edge choices
        #         edgeChoices = self.connection_info.outgoing_edges_dict[current_edge]
        #         # print(type(edgeChoices))
        #         # calculate the congestion ratio
        #         for key, value in edgeChoices.items():
        #             edgeLength = self.connection_info.edge_length_dict[value]
        #             # print("EDGE LENGTH = " + str(edgeLength))
        #             # print(str(edgeLength))
        #             carNum = self.connection_info.edge_vehicle_count[value]
        #             # print("CAR COUNT = " + str(carNum))
        #             congestionRatio = carNum/edgeLength
        #             # print("CONGESTION RATIO = " + str(congestionRatio))
        #             crList.append(congestionRatio)
        #             print(str(crList))
        #             # THERES NO SPEED :(
        #             # Get the speed of the current edge
        #             # speedLimit = self.connection_info.outgoing_edges_dict[current_edge].maxSpeed
        #             # print(str(speedLimit))

        #         # Calculate the shortest path plus the smallest congestion ratio

        #         # append that choice and value to the list
        #         # choose between shortest path or least congested lane
        #         # the choices from djikstra's aren't lowering the mean dealine that means there's no better path to choose
        #         # To prevent a traffic build up choose the lane with the lowest congestion ratio                                

        #         # Break out and append choice to choice list


            # # Putting this here for now - Can't test without this
            # choice = self.direction_choices[random.randint(0, 5)]
            # decision_list.append(choice)
            '''
            Your algo ends here
            '''
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

        # print(vId_list)
        return local_targets

    