package cz.agents.agentdrive.highway.creator;

import cz.agents.agentdrive.highway.agent.Agent;
import cz.agents.agentdrive.highway.environment.HighwayEnvironment;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.LocalSimulatorHandler;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.ModuleSimulatorHandler;
import cz.agents.agentdrive.highway.environment.roadnet.RoadNetworkRouter;
import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.storage.HighwayStorage;
import cz.agents.agentdrive.highway.storage.plan.PlansOut;
import cz.agents.agentdrive.simulator.lite.creator.SimulatorCreator;
import cz.agents.agentdrive.simulator.lite.environment.SimulatorEnvironment;
import cz.agents.alite.configreader.ConfigReader;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.simulation.Simulation;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import java.io.PrintWriter;
import java.util.*;

public class AgentDrive {

    private HighwayEnvironment highwayEnvironment;
    private RoadNetwork roadNetwork;
    private Integer time;
    Timer timer = new Timer();
    protected long timestep;
    private SimulatorEnvironment environment;
    private static String CONFIG_FILE = "settings/groovy/local/honza.groovy";
    protected String DEFAULT_CONFIG_FILE = "settings/groovy/config.groovy";

    private static String HIGHWAY_CONFIG_FILE;
    private static final Logger logger = Logger.getLogger(SimulatorCreator.class);

    public void init() {

        // Configuration loading using alite's Configurator and ConfigReader
        ConfigReader configReader = new ConfigReader();
        //configReader.loadAndMerge(DEFAULT_CONFIG_FILE);
        configReader.loadAndMerge(CONFIG_FILE);

        Configurator.init(configReader);


        String logfile = Configurator.getParamString("cz.highway.configurationFile", "settings/log4j/log4j.properties");
        PropertyConfigurator.configure(logfile);

        logger.setLevel(Level.INFO);
        logger.info("Configuration loaded from: " + CONFIG_FILE);
        if (logger.isDebugEnabled()) {
            logger.debug("Printing complete configuration on the System.out >>");
            configReader.writeTo(new PrintWriter(System.out));
        }
        logger.info("log4j logger properties loaded from: " + logfile);
        XMLReader xmlReader = new XMLReader();
        roadNetwork = xmlReader.parseNetwork(Configurator.getParamString("simulator.net.folder", "nets/junction-big/"));
        RoadNetworkRouter.setRoadNet(roadNetwork);
        Simulation simulation = new Simulation();
        simulation.setSimulationSpeed(0);
        time = 0;
        highwayEnvironment = new HighwayEnvironment(time, roadNetwork);
        //timer.scheduleAtFixedRate(new TimeUpdate(), 0, 1000);
        initTraffic();
    }

    public void getResults() {
        while (!HighwayStorage.isFinished) {
            highwayEnvironment.setCurrentTime(highwayEnvironment.getCurrentTime() + 1000);
            highwayEnvironment.getStorage().updateCars(highwayEnvironment.getStorage().getCurrentRadarData());
        }
        timer.cancel();
    }

    private void initTraffic() {
        final XMLReader reader = new XMLReader(Configurator.getParamString("simulator.net.folder", "notDefined"));
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
        highwayEnvironment.addSimulatorHandler(new LocalSimulatorHandler(highwayEnvironment, new HashSet<Integer>(plannedVehicles)));
    }

    class TimeUpdate extends TimerTask {

        public void run() {
            time += 1000;
            System.out.println("time update");
        }
    }

    public static void main(String[] args) {
        AgentDrive ad = new AgentDrive();
        ad.init();
        ad.getResults();
    }
}
