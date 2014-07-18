package cz.agents.highway.creator;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.configreader.ConfigReader;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.creator.Creator;
import cz.agents.alite.protobuf.communicator.Communicator;
import cz.agents.alite.protobuf.communicator.callback.ConnectCallback;
import cz.agents.alite.protobuf.factory.ProtobufFactory;
import cz.agents.alite.simulation.Simulation;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.*;
import cz.agents.highway.agent.Agent;
import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.environment.roadnet.XMLReader;
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
import cz.agents.highway.vis.RoadObjectLayer;
import cz.agents.highway.vis.SimulationControlLayer;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import javax.vecmath.Point2d;
import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;
import java.awt.*;
import java.io.IOException;
import java.util.*;
import java.util.List;

/**
 * Dash board like controller, that manages launching simulators and their synchronization with agentDrive
 * <p/>
 * Created by wmatex on 27.6.14.
 */
public class DashBoardController extends DefaultCreator implements EventHandler, Creator {
    private final Logger logger = Logger.getLogger(DashBoardController.class);

    /**
     * This class is responsible for sending simulator an appropriate plans and updates
     */
    private class SimulatorHandler {
        protected final ProtobufFactory factory;
        protected final Set<Integer> plannedVehicles;

        protected PlansOut plans = new PlansOut();

        protected SimulatorHandler(ProtobufFactory factory, Set<Integer> plannedVehicles) {
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

            for (int id : notPlanned) {
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

    private class LocalSimulatorHandler extends SimulatorHandler {

        private LocalSimulatorHandler(ProtobufFactory factory, Set<Integer> plannedVehicles) {
            super(factory, plannedVehicles);
        }

        @Override
        public void sendPlans(Map<Integer, RoadObject> vehicleStates) {
            RadarData radarData = new RadarData();

            Set<Integer> notPlanned = new HashSet<Integer>(vehicleStates.keySet());
            notPlanned.removeAll(plannedVehicles);

            for (int id : notPlanned) {
                radarData.add(vehicleStates.get(id));
            }

            // Finally send plans and updates

            executePlans(plans);


            plans.clear();
        }

        private void executePlans(PlansOut plans) {
            Map<Integer, RoadObject> currStates = highwayEnvironment.getStorage().getPosCurr();
            RadarData radarData = new RadarData();
            double duration = 0;
            for (Integer carID : plans.getCarIds()) {
                Collection<Action> plan = plans.getPlan(carID);
                RoadObject state = currStates.get(carID);
                for (Action action : plan) {
                    if (action.getClass().equals(WPAction.class)) {
                        WPAction wpAction = (WPAction) action;

                        Vector3f vel = new Vector3f(state.getPosition());
                        vel.negate();
                        vel.add(wpAction.getPosition());
                        duration = wpAction.getPosition().distance(state.getPosition()) / wpAction.getSpeed();
                        int lane = highwayEnvironment.getRoadNetwork().getLaneNum(wpAction.getPosition());
                        state = new RoadObject(carID, duration, lane, wpAction.getPosition(), vel);

                    }
                }
                radarData.add(state);
            }
            //send radar-data to storage with duration delay
            highwayEnvironment.getEventProcessor().addEvent(HighwayEventType.RADAR_DATA, highwayEnvironment.getStorage(), null, radarData, Math.max(1,(long) (duration * 1000)));
        }
    }


    /// Map of all running simulator processes
    private Map<String, Process> simulators = new HashMap<String, Process>();

    private Communicator communicator;

    private List<SimulatorHandler> simulatorHandlers = new LinkedList<SimulatorHandler>();

    @Override
    public void init(String[] args) {
        super.init(args);
        logger.setLevel(Level.INFO);
    }

    @Override
    public void create() {
        super.create();
        communicator = highwayEnvironment.getCommunicator();
        simulation.addEventHandler(this);
        // Finally start the simulation
        runSimulation();
    }


    /**
     * This method is responsible for even distribution of vehicles between all configured simulators
     */
    private void initTraffic() {
        final XMLReader reader = XMLReader.getInstance();
        // All vehicle id's
        final Collection<Integer> vehicles = reader.getRoutes().keySet();
        final int size = vehicles.size();
        final int simulatorCount = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).size();
        final HighwayStorage storage = highwayEnvironment.getStorage();
        Map<Integer, Point2f> initialPositions = reader.getInitialPositions();

        if (simulatorCount == 0) {
            //Simulatorlocal initialization - FIXME remove duplicate code with ConnectCallback.invoke()

            Iterator<Integer> vehicleIt = vehicles.iterator();
            RadarData update = new RadarData();
            Map<Integer, Agent> agents = storage.getAgents();

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
                Point2f pos = initialPositions.get(vehicleID);
                Point3f initialPosition = agent.getInitialPosition();
                if (pos != null) {
                    initialPosition.setX(pos.x);
                    initialPosition.setY(pos.y);
                }
                // FIXME!!!
                Vector3f initialVelocity = new Vector3f(1, 1, 0);

                int lane = highwayEnvironment.getRoadNetwork().getLaneNum(initialPosition);
                update.add(new RoadObject(vehicleID, 0d, lane, initialPosition, initialVelocity));

            }
            simulatorHandlers.add(new LocalSimulatorHandler(null, new HashSet<Integer>(vehicles)));
            storage.updateCars(update);
        }

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
                Map<Integer, Point2f> initialPositions = reader.getInitialPositions();

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

                    Point2f pos = initialPositions.get(vehicleID);
                    Point3f initialPosition = agent.getInitialPosition();
                    if (pos != null) {
                        initialPosition.setX(pos.x);
                        initialPosition.setY(pos.y);
                    }
                    // FIXME!!!
                    Vector3f initialVelocity = new Vector3f(1, 1, 0);
                    int lane = highwayEnvironment.getRoadNetwork().getLaneNum(initialPosition);

                    // If the simulator should simulate this vehicle
                    // This divides the id's to even parts
                    if (i < section * size / simulatorCount && i >= (section - 1) * size / simulatorCount) {
                        plans.addAction(new WPAction(vehicleID, 0d, initialPosition, initialVelocity.length()));
                        plannedVehicles.add(vehicleID);
                    } else {
                        update.add(new RoadObject(vehicleID, 0d, lane, initialPosition, initialVelocity));
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

    @Override
    public void runSimulation() {
        // Start all configured simulators
        List<String> simulatorsToRun = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class);
        logger.info("Running this simulators: " + simulatorsToRun);
        try {
            initTraffic();
            for (String sim : simulatorsToRun) {
                runSimulator(sim);
            }
            if (simulatorsToRun.size() >= 1) {
                synchronized (simulation) {
                    try {
                        simulation.wait();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
            simulation.run();
        } catch (IOException e) {
            logger.error("Failed to run a simulator: " + e);
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

            for (SimulatorHandler handler : simulatorHandlers) {
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
        String launchScript = Configurator.getParamString("highway.dashboard.simulators." + name + ".launch", "");
        String[] parts = launchScript.split(" ");
        logger.info("Starting simulator: " + name + ", script: " + launchScript);
        if (!launchScript.isEmpty()) {
            simulators.put(name, new ProcessBuilder().inheritIO().command(parts).start());
        }
    }

}
