import traci
import os, sys
import environment_state

if 'SUMO_HOME' not in os.environ:
	print "SUMO setup incomplete. Exiting."
	exit()

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)

sumoBinary = "C:/Program Files (x86)/DLR/Sumo/bin/sumo-gui"
sumoCmd = [sumoBinary, "-c", "C:/Users/sreeniva/Desktop/Reinforcement Learning/madrl_traffic_control/Sumo Stuff/hello.sumocfg"]

traci.start(sumoCmd)

# run the entire RL workflow
environment_state.test_workflow()
