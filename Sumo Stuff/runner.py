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

import os
import sys
import optparse
import subprocess
import random


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

"""
This calculates the Total delay for every phase and stores it in a list
for the entire iteration of the the session
"""
def mean(numbers):
    return float(sum(numbers)) / max(len(numbers), 1)

Max_Value_Lane_lr = []

def get_waiting_time(flag):
    if traci.trafficlights.getPhase("2") == 0:
        Max_Value_0 = traci.lane.getWaitingTime("left-right-1_0")
        Max_Value_1 = traci.lane.getWaitingTime("left-right-1_1")
        Max_Value_2 = traci.lane.getWaitingTime("left-right-1_2")
        Max_Value_3 = traci.lane.getWaitingTime("left-right-1_3")
        Max_Value_10 = traci.lane.getWaitingTime("right-left-1_0")
        Max_Value_11 = traci.lane.getWaitingTime("right-left-1_1") 
        Max_Value_12 = traci.lane.getWaitingTime("right-left-1_2")
        Max_Value_13 = traci.lane.getWaitingTime("right-left-1_3")
        print("Max_Value_13 = ", Max_Value_0)
        flag = 1
    if traci.trafficlights.getPhase("2") == 2:
        if flag == 1:
            print (Max_Value_0,Max_Value_1,Max_Value_2,
                Max_Value_3,Max_Value_10,Max_Value_11,Max_Value_12,
                Max_Value_13)
            Max = mean([Max_Value_0,Max_Value_1,Max_Value_2,
                Max_Value_3,Max_Value_10,Max_Value_11,Max_Value_12,
                Max_Value_13])
            Max_Value_Lane_lr.append(Max)
            flag = 0
        step += 1
    print (Max_Value_Lane_lr)


def run():
    """execute the TraCI control loop"""
    step = 0
    Max_Value_Lane_lr = []
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()
        if traci.trafficlights.getPhase("2") == 0:
            Max_Value_lr_0 = traci.lane.getWaitingTime("left-right-1_0")
            print (Max_Value_lr_0)
            Max_Value_lr_1 = traci.lane.getWaitingTime("left-right-1_1")
            Max_Value_lr_2 = traci.lane.getWaitingTime("left-right-1_2")
            Max_Value_lr_3 = traci.lane.getWaitingTime("left-right-1_3")
            Max_Value_10 = traci.lane.getWaitingTime("right-left-1_0")
            Max_Value_11 = traci.lane.getWaitingTime("right-left-1_1") 
            Max_Value_12 = traci.lane.getWaitingTime("right-left-1_2")
            Max_Value_13 = traci.lane.getWaitingTime("right-left-1_3")
            flag = 1
        if traci.trafficlights.getPhase("2") == 2:
            if flag == 1:
                print (Max_Value_0,Max_Value_1,Max_Value_2,
                    Max_Value_3,Max_Value_10,Max_Value_11,Max_Value_12,
                    Max_Value_13)
                Max = mean([Max_Value_0,Max_Value_1,Max_Value_2,
                    Max_Value_3,Max_Value_10,Max_Value_11,Max_Value_12,
                    Max_Value_13])
                Max_Value_Lane_lr.append(Max)
                flag = 0
            Max_Value_0 = traci.lane.getWaitingTime("up-down-1_0")
            Max_Value_1 = traci.lane.getWaitingTime("up-down-1_1")
            Max_Value_2 = traci.lane.getWaitingTime("up-down-1_2")
            Max_Value_3 = traci.lane.getWaitingTime("up-down-1_3")
            Max_Value_10 = traci.lane.getWaitingTime("down-up-1_0")
            Max_Value_11 = traci.lane.getWaitingTime("down-up-1_1") 
            Max_Value_12 = traci.lane.getWaitingTime("down-up-1_2")
            Max_Value_13 = traci.lane.getWaitingTime("down-up-1_3")
        step += 1
        print (Max_Value_Lane_lr)
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
    traci.start([sumoBinary,"-c", "hello.sumocfg",
    "--tripinfo-output", "tripinfo.xml","--additional-files","hello.det.xml"])
    run()
