#!/usr/bin/env python
"""
@file    runner.py
@author  Lena Kalleske
@author  Daniel Krajzewicz
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2009-03-26
@version $Id: runner.py 24864 2017-06-23 07:47:53Z behrisch $

Tutorial for traffic light control via the TraCI interface.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2009-2017 DLR/TS, Germany

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division
import os
import sys
import optparse
import subprocess
import random
from collections import defaultdict
import csv
# we need to import python modules from the $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(
        __file__), '..', '..', '..', '..', "tools"))  # tutorial in tests
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(__file__), "..", "..", "..")), "tools"))  # tutorial in docs
    from sumolib import checkBinary  # noqa
except ImportError:
    sys.exit(
        "please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

import traci

names_incoming_lanes = ["left-right-1_0","left-right-1_1",
"left-right-1_2","left-right-1_3","right-left-1_0",
"right-left-1_1","right-left-1_2","right-left-1_3",
"up-down-1_0","up-down-1_1","up-down-1_2","up-down-1_3",
"down-up-1_0","down-up-1_1","down-up-1_2","down-up-1_3"]

"""
Code to place the detectors on the lanes. We pass the entire series of incoming lanes place detectors
at every 5 meter length gap.
"""
def generate_detectorfile():
    with open("hello.det.xml","w") as detectors:
        print("""<additional>""",file=detectors)
        for idx, i in enumerate(names_incoming_lanes):
            num = 80
            count = 0
            while num>40:
                print('<e1Detector id="%d" lane="%s" pos="%d" freq="30" file="hello.out" friendlyPos="x"/>'
                 % (idx*100+count,i,num),file=detectors)
                num = num-5
                count = count + 1
        print('</additional>',file=detectors)

def generate_routefile():
    random.seed(42)  # make tests reproducible
    N = 100000  # number of time steps
    # demand per second from different directions
    pWE = 1. / 10
    pEW = 1. / 11
    pNS = 1. / 30
    with open("data/cross.rou.xml", "w") as routes:
        print("""<routes>
        <vType id="typeWE" accel="0.8" decel="4.5" sigma="0.5" length="5" minGap="2.5" maxSpeed="16.67" guiShape="passenger"/>
        <vType id="typeNS" accel="0.8" decel="4.5" sigma="0.5" length="7" minGap="3" maxSpeed="25" guiShape="bus"/>

        <route id="right" edges="51o 1i 2o 52i" />
        <route id="left" edges="52o 2i 1o 51i" />
        <route id="down" edges="54o 4i 3o 53i" />""", file=routes)
        lastVeh = 0
        vehNr = 0
        for i in range(N):
            if random.uniform(0, 1) < pWE:
                print('<vehicle id="right_%i" type="typeWE" route="right" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pEW:
                print('<vehicle id="left_%i" type="typeWE" route="left" depart="%i" />' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
            if random.uniform(0, 1) < pNS:
                print('<vehicle id="down_%i" type="typeNS" route="down" depart="%i" color="1,0,0"/>' % (
                    vehNr, i), file=routes)
                vehNr += 1
                lastVeh = i
        print("</routes>", file=routes)

# The program looks like this
#    <tlLogic id="0" type="static" programID="0" offset="0">
# the locations of the tls are      NESW
#        <phase duration="31" state="GrGr"/>
#        <phase duration="6"  state="yryr"/>
#        <phase duration="31" state="rGrG"/>
#        <phase duration="6"  state="ryry"/>
#    </tlLogic>


import pandas as pd
from matplotlib import pyplot as plt
def Get_Average_Waiting_Timestep(a,b):
    avg_wait = []
    for idx,i in enumerate(a):
        if b[idx] == 0:
            avg_wait.append(0)
        else:
            avg_wait.append(a[idx]/b[idx])
    return avg_wait

def plotthedata(a):
    plt.plot(a)
    plt.ylabel("Average delay per vehicle")
    plt.xlabel("Time")
    plt.show()


def get_lane_coordinates():
    total = []
    for idx,i in enumerate(names_incoming_lanes):
        if idx > 0 and idx%4 != 0:
            lane = []
            a,b=traci.lane.getShape(names_incoming_lanes[idx-1])
            c,d=traci.lane.getShape(names_incoming_lanes[idx])
            ax,ay = a
            ax = int(ax)
            ay = int(ay)
            zippeda = (ax,ay)
            bx,by = b
            bx = int(bx)
            by = int(by)
            zippedb = (bx,by)
            cx,cy = c
            cx = int(cx)
            cy = int(cy)
            zippedc = (cx,cy)
            dx,dy = d
            dx = int(dx)
            dy = int(dy)
            zippedd = (dx,dy)
            lane.append(zippeda)
            lane.append(zippedb)
            lane.append(zippedc)
            lane.append(zippedd)
            # print(lane)
            total.append(lane)
    print(len(total))
    return total

# colr = [(0,86),(86,86),(0,100),(86,100)]
# # codu = [(100,0),(114,0),(100,86),(114,86)]
# # corl = [(200,100),(200,113),(113,100),(113,113)]
# # coud = [(86,200),(100,200),(86,113),(100,113)]
#
# all_coordinates = [colr,codu,corl,coud]


def initialize_matrix():
    notalane = [-1]
    w, h = 200, 200;
    Matrix = [[zip(notalane,notalane) for x in range(w)] for y in range(h)]
    return Matrix



def extract_lanes(Matrix,t):
    count = 0
    listoflanes = []
    for lane_coordinate in t:
        count = count + 1
        a,b,c,d = lane_coordinate
        ax,ay = a
        cx,cy = c
        bx,by = b
        if(count<=6):
            inner = (ay,cy)
            outer = (ax,bx)
            inner = sorted(inner)
            outer = sorted(outer)
        else:
            inner = (ax,cx)
            outer = (ay,by)
            inner = sorted(inner)
            outer = sorted(outer)
        lane = [row[outer[0]:outer[1]] for row in Matrix[inner[0]:inner[1]]]
        # print(len(lane))
        for idx,i in enumerate(lane):
            flat_list = [item for sublist in i for item in sublist]
            lane[idx] = flat_list
            # print(max(lane[:,2]))
        col = 0
        Max_column = [max(a,key=lambda x:x[1]) for a in zip(*lane)]
        Max_row = [max(Max_column[n:n+5],key=lambda x:x[1]) for n in range(0, len(Max_column), 5)]
        print(Max_row)
        # exit()
        # subList = [max(A[i][n:n+2],key=lambda x:x[1]) for i in range(0,len(A)) for n in range(0, len(A[0]), 2)]
        # print subList
        # print (Max_column)
        listoflanes.append(Max_row)
        # maxtuple = ()
        # while(col<len(lane[0])-1):
        #     column = []
        #     for idx,i in enumerate(lane):
        #         column.append(i[col])
        #     maximum = -1
        #     for i in column:
        #         if(i[1]>max):
        #             maxtuple = i
        #     col = col + 1
        #     Max_column.append(maxtuple)

        # print(Max_column)
                # ele = lane[i][]
                # column.extend()
        # val_ = [x[0] for i in lane for x in i]
        # print(max(val_))
        # print(lane)
        # print("Lane[0] = {},Lane[len] = {}".format(lane[1:6,0],lane[len(lane)-1]))
        # print(lane[])
    print (listoflanes)
    # exit()
    # print(print()

    return listoflanes
        # choice = int(len(lane)/2)
        # print(lane)
    # print("\n\n\n")
    # print(listoflanes)
# def extract_lanes(Matrix):
#     lane_values = []
#     count = 0
#     all_coordinates = get_lane_coordinates()
#     for lane_coordinate in all_coordinates:
#         print("Lane coordinates = ",lane_coordinate)
#         x = []
#         y = []
#         for ithcordinate in lane_coordinate:
#             x_cord,y_cord = ithcordinate
#             x.append(x_cord)
#             y.append(y_cord)
#             xset = set(x)
#             yset = set(y)
#             xset = sorted(xset)
#             yset = sorted(yset)
#         lane = [row[xset[0]:xset[1]] for row in Matrix[yset[0]:yset[1]]]
#         print(len(lane))
#         increment = 3
#         increment_ = 2
#         if count%2 == 0:
#             values = []
#             while increment<len(lane):
#                 while increment_<len(lane[increment]):
#                     values.append(lane[increment][increment_])
#                     increment_ = increment_ + 5
#                 increment = increment + 3
#             lane_values.append(values)
#             # selecting lanes y values --> lanes
#         else:
#             values = []
#             # selecting lanes x values --> lanes
#             lane = map(list,map(None,*lane))
#             while increment<len(lane):
#                 while increment_<len(lane[increment]):
#                     values.append(lane[increment][increment_])
#                     increment_ = increment_ + 5
#                 increment = increment + 3
#             lane_values.append(values)
#         # print(lane_values)

def get_vehicle_info(Matrix,t):
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        present = [1]
        absent = [0]
        count = 0
        if step%2 == 0:
            # Matrix = initialize_matrix()
            xpos = []
            ypos = []
            for veh_id in traci.vehicle.getIDList():
                position = traci.vehicle.getPosition(veh_id)
                speed = traci.vehicle.getSpeed(veh_id)
                speed = [round(speed,3)]
                x,y = position
                count = count + 1
                x=int(x)
                y=int(y)
                if count <4:
                    xpos.append(x)
                    ypos.append(y)
                Matrix[x][y] = zip(present,speed)
                # print(x,y,Matrix[x][y])
                # print("Position of x ={}, y={} value = {} ".format(x,y,speed))

            data=[('smith, bob',2),('carol',3),('ted',4),('alice',5)]
            extracted_lanes = extract_lanes(Matrix,t)
            # filename = "/home/newuser/Jayanth/madrl_traffic_control-master/Data/extract_lanes.csv"
            # # with open(filename, 'a') as myfile:
            # #     wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
            # #     wr.writerow(['presence','speed'])
            # #     for row in extracted_lanes:
            # #         wr.writerow(row)
        step += 1

def run2():
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if step%5 == 0:
            speed = [[] for _ in range(len(names_incoming_lanes))]
            for idx,i in enumerate(names_incoming_lanes):
                count = 0
                num = 80
                while num>40:
                    detectorid = idx*100+count
                    speed[idx].append(traci.inductionloop.getLastStepMeanSpeed(str(detectorid)))
                    num = num-5
                    count = count + 1
            for idx,i in enumerate(names_incoming_lanes):
                filename = "/home/newuser/Jayanth/madrl_traffic_control-master/Sumo Stuff/Data/"+i+".csv"
                with open(filename, 'a') as myfile:
                    wr = csv.writer(myfile, quoting=csv.QUOTE_ALL)
                    wr.writerow(speed[idx])
        step += 1



def run():
    Waiting_Time = pd.DataFrame()
    append_data = []
    Sum_Waiting_Time = []
    listoflistwait = []
    listoflistveh = []
    step = 0
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        list = []
        num_veh = []
        for idx,i in enumerate(names_incoming_lanes):
            list.append(traci.lane.getWaitingTime(i))
            num_veh.append(traci.lane.getLastStepVehicleNumber(i))
        listoflistwait.append(list)
        listoflistveh.append(num_veh)
        step += 1
    Waiting_Time = pd.DataFrame(listoflistwait)
    Waiting_Time.columns = names_incoming_lanes
    Num_Vehicles = pd.DataFrame(listoflistveh)
    Sum_Waiting_Time = Waiting_Time.sum(axis=1)
    Sum_Waiting_Time.tolist()
    Sum_Num_Vechicles = Num_Vehicles.sum(axis=1)
    Sum_Num_Vechicles.tolist()
    print(type(Sum_Num_Vechicles))
    # print(Sum_Waiting_Time)
    Average_Wait = Get_Average_Waiting_Timestep(Sum_Waiting_Time,Sum_Num_Vechicles)
    plotthedata(Average_Wait)
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
    # generate_detectorfile()
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary,"-c", "hello.sumocfg","--tripinfo-output", "tripinfo.xml"])
    Matrix = initialize_matrix()
    t = get_lane_coordinates()
    State_Space = get_vehicle_info(Matrix,t)
    # get_lane_coordinates()
    # run2()
