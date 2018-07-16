package cz.agents.agentdrive.highway.creator;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.configreader.ConfigReader;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.simulation.SimulationEventType;

import cz.agents.agentdrive.highway.agent.Agent;
import cz.agents.agentdrive.highway.environment.HighwayEnvironment;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.ModuleSimulatorHandler;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.PlanCallback;

import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.highway.storage.HighwayEventType;
import cz.agents.agentdrive.highway.storage.HighwayStorage;
import cz.agents.agentdrive.highway.storage.RadarData;
import cz.agents.agentdrive.highway.storage.plan.PlansOut;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import java.io.PrintWriter;
import java.util.*;

public class AgentDrive extends DefaultCreator implements EventHandler, Observer, Runnable {
    private final Logger logger = Logger.getLogger(AgentDrive.class);
    protected long timestep;
    private PlanCallback plancallback;

    public AgentDrive(String configFileLocation)
    {
        CONFIG_FILE = configFileLocation;
    }

    public void run(){
        init();
        create();
    }

    public void init()
    {
        ConfigReader configReader = new ConfigReader();
        configReader.loadAndMerge(DEFAULT_CONFIG_FILE);
        configReader.loadAndMerge(CONFIG_FILE);

        Configurator.init(configReader);


        String logfile = Configurator.getParamString("cz.highway.configurationFile", "settings/log4j/log4j.properties");
        PropertyConfigurator.configure(logfile);

        logger.setLevel(Level.INFO);
        logger.info("Configuration loaded from: " + CONFIG_FILE);
        if(logger.isDebugEnabled()){
            logger.debug("Printing complete configuration on the System.out >>");
            configReader.writeTo(new PrintWriter(System.out));
        }
        logger.info("log4j logger properties loaded from: " + logfile);
    }
    @Override
    public void create() {
        timestep = Configurator.getParamInt("highway.timestep", 100);
        super.create();
        simulation.addEventHandler(this);
        runSimulation();
    }


    /**
     * This method is responsible for even distribution of vehicles between all configured simulators
     */
    private void initTraffic() {
        final XMLReader reader = new XMLReader(Configurator.getParamString("highway.net.folder", "notDefined"));
        // All vehicle id's
        final Collection<Integer> vehicles = reader.getRoutes().keySet();
        final Map<Integer, Float> departures = reader.getDepartures();
        // final int size = vehicles.size();
        final int size;
        if (!Configurator.getParamBool("highway.dashboard.sumoSimulation", true)) {
            size = Configurator.getParamInt("highway.dashboard.numberOfCarsInSimulation", vehicles.size());
        } else {
            size = vehicles.size();
        }
        final int simulatorCount = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).size();
        final HighwayStorage storage = highwayEnvironment.getStorage();

        Iterator<Integer> vehicleIt = vehicles.iterator();
        PlansOut plans = new PlansOut();
        //   RadarData update = ;
        Map<Integer, Agent> agents = storage.getAgents();
        Set<Integer> plannedVehiclesLocal = new HashSet<Integer>();
        int sizeL = size;
        if (size > vehicles.size()) sizeL = vehicles.size();

        for (int i = 0; i < sizeL; i++) {
            int vehicleID = vehicleIt.next();
            if (Configurator.getParamBool("highway.dashboard.sumoSimulation", true)) {
                storage.addForInsert(vehicleID, departures.get(vehicleID));
            } else {
                storage.addForInsert(vehicleID);
            }
            plannedVehiclesLocal.add(vehicleID);
        }
        final Set<Integer> plannedVehicles = plannedVehiclesLocal;
        highwayEnvironment.addSimulatorHandler(new ModuleSimulatorHandler(highwayEnvironment, new HashSet<Integer>(plannedVehicles), plancallback));
    }

    @Override
    public void runSimulation() {
        initTraffic();
        simulation.run();
    }

    @Override
    public EventProcessor getEventProcessor() {
        return simulation;
    }


    @Override
    public void handleEvent(Event event) {
        if (event.isType(SimulationEventType.SIMULATION_STARTED)) {
            System.out.println("Caught SIMULATION_STARTED from AgentDrive");
            //  highwayEnvironment.getStorage().updateCars(new RadarData());
            getEventProcessor().addEvent(HighwayEventType.TIMESTEP, null, null, null, timestep);
        } else if (event.isType(HighwayEventType.TIMESTEP)) {
            //if (isProtobufOn) communicator.run();
           getEventProcessor().addEvent(HighwayEventType.TIMESTEP, null, null, null, timestep);
        }
    }
    public void registerPlanCallback(PlanCallback plancallback)
    {
        this.plancallback = plancallback;
    }

    @Override
    public void update(Observable o, Object arg) {
        highwayEnvironment.getStorage().updateCars(((RadarData)arg));
    }
    public HighwayEnvironment getHighwayEnvironment(){
        return highwayEnvironment;
    }
}
