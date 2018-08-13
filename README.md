# AgentDrive

##Branches
- Master - main branch with the latest stable code.
- Multisim - supports multiple simulators
- Other stale branches archived by tags

#Documentation AgentDrive
##How to run demo
Run cz.agents.agentdrive.highway.Main class with arguments:
cz.agents.agentdrive.simulator.lite.creator.SimulationCreator settings/groovy/highway.groovy

where the first argument is Creator to be used and second argument is configuration file.

If the second argument is not used the default config file is used i.e. settings/groovy/highway.groovy. It is recommended not to change the default config file if not necessary, you can create a copy of it in subfolder 'local' and use it instead.

##How to create and use new networks
It is required to have installed SUMO tools (http://sumo.dlr.de/wiki/Downloads)

Get your osm file from: https://www.openstreetmap.org/
#####Using command line
- Use netconvert tool to create .net.xml file from osm representing your network. It is highly recommended to use these arguments:

    --tls.join --no-internal-links --no-turnarounds

    More info here: http://sumo.dlr.de/wiki/NETCONVERT

- To create random origins and destinations for agents use python script randomTrips.py, which is part of SUMO tools package.
    
    More info here: http://sumo.dlr.de/wiki/Tools/Trip

- Finally, to create path from generated origins and destinations use duarouter tool
    
    More info here: http://sumo.dlr.de/wiki/DUAROUTER

Prepared scripts which can also be used as examples are located in data/nets folder.
To use generatorWin.bat you have to set path to environment variable SUMO_HOME. First argument is name of a folder with osm file with a same name (excluding suffix).
#####Using netedit tool
Netedit is a great SUMO tool with GUI for visualizing, editing and creating *.net.xml files. Netedit is easy to use, but to use created networks in AgentDrive it is required to go to processing -> options and in TLS building tab check tls.join option and in processing tab check no-internal-links and no-turnarounds options.
More info: http://sumo.dlr.de/wiki/NETEDIT

I highly recommend to read https://dspace.cvut.cz/bitstream/handle/10467/61945/F3-BP-2015-Kubesa-David-Collision%20Avoidance%20on%20General%20Road%20Network.pdf to get better understanding.