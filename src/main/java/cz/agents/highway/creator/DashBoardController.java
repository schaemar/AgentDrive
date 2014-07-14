package cz.agents.highway.creator;

import java.awt.*;
import java.io.IOException;
import java.net.URI;
import java.util.*;
import java.util.List;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.configreader.ConfigReader;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.creator.Creator;
import cz.agents.alite.protobuf.communicator.Communicator;
import cz.agents.alite.protobuf.communicator.ServerCommunicator;
import cz.agents.alite.protobuf.communicator.callback.ConnectCallback;
import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.alite.protobuf.factory.ProtobufFactory;
import cz.agents.alite.simulation.Simulation;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.alite.transport.SocketTransportLayer;
import cz.agents.alite.transport.TransportLayerInterface;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.*;
import cz.agents.highway.agent.Agent;
import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.environment.roadnet.XMLReader;
import cz.agents.highway.protobuf.generated.simplan.MessageContainer;
import cz.agents.highway.protobuf.generated.dlr.DLR_MessageContainer;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.HighwayStorage;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.PlansOut;
import cz.agents.highway.storage.plan.WPAction;
import cz.agents.highway.util.Utils;
import cz.agents.highway.vis.NetLayer;
import cz.agents.highway.vis.ProtobufVisLayer;
import cz.agents.highway.vis.SimulationControlLayer;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import javax.vecmath.Point2d;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

/**
 * Dash board like controller, that manages launching simulators and their synchronization with agentDrive
 *
 * Created by wmatex on 27.6.14.
 */
public class DashBoardController implements EventHandler, Creator {
    private final Logger logger = Logger.getLogger(DashBoardController.class);

    /**
     * This class is responsible for sending simulator an appropriate plans and updates
     */
    private class SimulatorHandler {
        private final ProtobufFactory factory;
        private final Set<Integer> plannedVehicles;

        private PlansOut plans = new PlansOut();

        private SimulatorHandler(ProtobufFactory factory, Set<Integer> plannedVehicles) {
            this.factory = factory;
            this.plannedVehicles = plannedVehicles;
        }

        public boolean hasVehicle(int vehicleID) {
            return plannedVehicles.contains(vehicleID);
        }

        public void addAction(Action action) {
            plans.addAction(action);
        }

        public boolean isReady() {
            return plans.getCarIds().size() >= plannedVehicles.size();
        }

        public void sendPlans(Map<Integer, RoadObject> vehicleStates) {
            RadarData radarData = new RadarData();

            Set<Integer> notPlanned = new HashSet<Integer>(vehicleStates.keySet());
            notPlanned.removeAll(plannedVehicles);

            for (int id: notPlanned) {
                radarData.add(vehicleStates.get(id));
            }

            // Finally send plans and updates
            try {
                factory.send(plans);
                factory.send(radarData);
            } catch (IOException e) {
                e.printStackTrace();
            }

            plans.clear();
        }

    }

    // Constants
    private static final String CONFIG_FILE = "settings/groovy/highway.groovy";
    /// Map of all running simulator processes
    private Map<String, Process> simulators = new HashMap<String, Process>();

    private Communicator communicator;
    private Simulation simulation = null;
    private HighwayEnvironment highwayEnvironment = null;

    private List<SimulatorHandler> simulatorHandlers = new LinkedList<SimulatorHandler>();

    @Override
    public void init(String[] args) {
        // Configuration loading using Alite's Configurator and ConfigReader
        ConfigReader configReader = new ConfigReader();
        configReader.loadAndMerge(CONFIG_FILE);
        Configurator.init(configReader);

        String logfile = Configurator.getParamString("cz.highway.configurationFile",
                "settings/log4j/log4j.properties");
        PropertyConfigurator.configure(logfile);

        logger.info("Configuration loaded from: " + CONFIG_FILE);
        logger.info("log4j logger properties loaded from: " + logfile);
        logger.setLevel(Level.INFO);

    }

    @Override
    public void create() {
        double simulationSpeed = Configurator.getParamDouble("highway.simulationSpeed", 1.0);
        logger.info("Simulation speed: " + simulationSpeed);

        logger.info("\n>>> SIMULATION CREATION\n");
        simulation = new Simulation();

        logger.info("\n>>> COMMUNICATOR CREATION\n");

        logger.info("\n>>> ENVIRONMENT CREATION\n");
        highwayEnvironment = new HighwayEnvironment(simulation);
        communicator = highwayEnvironment.getCommunicator();

        simulation.addEventHandler(this);

        if (Configurator.getParamBool("highway.vis.isOn", false)) {
            logger.info("\n>>> VISUALISATION CREATION\n");
            createVisualization();
            VisManager.registerLayer(new NetLayer(highwayEnvironment.getRoadNetwork()));
            VisManager.registerLayer(ProtobufVisLayer.create(highwayEnvironment.getStorage()));
            VisManager.registerLayer(SimulationControlLayer.create(simulation));
        }
        simulation.setSimulationSpeed(simulationSpeed);

        // Finally start the simulation
        runSimulation();
    }

    private void createVisualization() {
        logger.info(">>> VISUALIZATION CREATED");

        VisManager.setInitParam("Highway Protobuf Operator", 1024, 768);
        VisManager.setSceneParam(new VisManager.SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(0, 0);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 0.75;
            }
        });

        VisManager.init();

        // Overlay

        VisManager.registerLayer(ColorLayer.create(Color.LIGHT_GRAY));
        VisManager.registerLayer(VisInfoLayer.create());
        VisManager.registerLayer(FpsLayer.create());
        VisManager.registerLayer(LogoLayer.create(Utils.getResourceUrl("img/atg_blue.png")));
        VisManager.registerLayer(HelpLayer.create());

    }

    /**
     * This method is responsible for even distribution of vehicles between all configured simulators
     */
    private void initTraffic() {
        XMLReader reader = XMLReader.getInstance();
        // All vehicle id's
        final Collection<Integer> vehicles = reader.getRoutes().keySet();
        final int size = vehicles.size();
        final int simulatorCount = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).size();
        final HighwayStorage storage = highwayEnvironment.getStorage();

        // Divide vehicles evenly to the simulators
        communicator.registerConnectCallback(new ConnectCallback() {
            private int section = 1;

            @Override
            public void invoke(ProtobufFactory factory) {
                Iterator<Integer> vehicleIt = vehicles.iterator();
                PlansOut plans = new PlansOut();
                RadarData update = new RadarData();
                Map<Integer, Agent> agents = storage.getAgents();
                Set<Integer> plannedVehicles = new HashSet<Integer>();


                // Iterate over all configured vehicles
                for (int i = 0; i < size; i++) {
                    int vehicleID = vehicleIt.next();
                    // Create agent for every single vehicle
                    Agent agent;
                    if (agents.containsKey(vehicleID)) {
                        agent = agents.get(vehicleID);
                    } else {
                        agent = storage.createAgent(vehicleID);
                    }

                    Point3f initialPosition = agent.getInitialPosition();
                    // FIXME!!!
                    Vector3f initialVelocity = new Vector3f(1,1,0);

                    // If the simulator should simulate this vehicle
                    // This divides the id's to even parts
                    if (i < section*size/simulatorCount && i >= (section-1)*size/simulatorCount) {
                        plans.addAction(new WPAction(vehicleID, 0d, initialPosition, initialVelocity.length()));
                        plannedVehicles.add(vehicleID);
                    } else {
                        update.add(new RoadObject(vehicleID, 0d, 0, initialPosition, initialVelocity));
                    }
                }

                // Create new simulator handler
                simulatorHandlers.add(new SimulatorHandler(factory, plannedVehicles));

                try {
                    // Send client the 'init'
                    factory.send(plans);
                    factory.send(update);
                } catch (IOException e) {
                    e.printStackTrace();
                }

                // This is the last simulator, start the simulation
                if (section >= simulatorCount) {
                    synchronized (simulation) {
                        simulation.notify();
                    }
                }
                // Increase the section so the next simulator will simulate different vehicles
                section++;
            }
        });
    }

    public void runSimulation() {
        // Start all configured simulators
        List<String> simulatorsToRun = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class);
        logger.info("Running this simulators: "+simulatorsToRun);
        try {
            initTraffic();
            for (String sim : simulatorsToRun) {
                runSimulator(sim);
            }
            synchronized (simulation) {
                try {
                    simulation.wait();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            simulation.run();
        } catch (IOException e) {
            logger.error("Failed to run a simulator: "+e);
            e.printStackTrace();
        }
    }

    @Override
    public EventProcessor getEventProcessor() {
        return simulation;
    }

    @Override
    public void handleEvent(Event event) {
        if (event.isType(SimulationEventType.SIMULATION_STARTED)) {
            System.out.println("Caught SIMULATION_STARTED from DashBoard");
        } else if (event.isType(HighwayEventType.NEW_PLAN)) {
            Action action = (Action) event.getContent();

            for (SimulatorHandler handler: simulatorHandlers) {
                if (handler.hasVehicle(action.getCarId())) {
                    handler.addAction(action);
                }
                if (handler.isReady()) {
                    handler.sendPlans(highwayEnvironment.getStorage().getPosCurr());
                }
            }
        }
    }

    /**
     * Run defined simulator
     *
     * @param name Name of the simulator as defined in configuration file
     * @throws IOException when launch script of the simulator cannot be executed
     */
    private void runSimulator(String name) throws IOException {
        String launchScript = Configurator.getParamString("highway.dashboard.simulators."+name+".launch", "");
        String[] parts = launchScript.split(" ");
        logger.info("Starting simulator: "+name+", script: "+launchScript);
        if (!launchScript.isEmpty()) {
            simulators.put(name, new ProcessBuilder().inheritIO().command(parts).start());
        }
    }

}
