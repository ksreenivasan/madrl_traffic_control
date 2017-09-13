# useful commands

# to create the network
netconvert --node-files=hello.nod.xml --edge-files=hello.edg.xml --output-file=hello.net.xml

# to run sim in command line mode
sumo -c hello.sumocfg

# to run sim in gui
sumo-gui -c hello.sumocfg

# Traci Commands
LaneDomain
getLastStepHaltingNumber(self, laneID)
getLastStepHaltingNumber(string) -> integer
 
Returns the total number of halting vehicles for the last time step on the given lane.
A speed of less than 0.1 m/s is considered a halt.

For normalization later on
getMaxSpeed(self, laneID)
getMaxSpeed(string) -> double
 
Returns the maximum allowed speed on the lane in m/s.

getWaitingTime
for each lane.

getAccumulatedWaitingTime
this is in vechicle domain