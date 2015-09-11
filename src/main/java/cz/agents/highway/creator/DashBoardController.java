package cz.agents.highway.creator;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.creator.Creator;
import cz.agents.alite.protobuf.communicator.Communicator;
import cz.agents.alite.protobuf.communicator.callback.ConnectCallback;
import cz.agents.alite.protobuf.factory.ProtobufFactory;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.highway.agent.Agent;
import cz.agents.highway.environment.SimulatorHandlers.LocalSimulatorHandler;
import cz.agents.highway.environment.SimulatorHandlers.ProtobufSimulationHandler;
import cz.agents.highway.environment.SimulatorHandlers.SimulatorHandler;
import cz.agents.highway.environment.roadnet.XMLReader;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.HighwayStorage;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.PlansOut;
import cz.agents.highway.storage.plan.WPAction;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;
import java.io.IOException;
import java.util.*;

/**
 * Dash board like controller, that manages launching simulators and their synchronization with agentDrive
 * <p/>
 * Created by wmatex on 27.6.14.
 */
public class DashBoardController extends DefaultCreator implements EventHandler, Creator {
    private final Logger logger = Logger.getLogger(DashBoardController.class);
    /**
     * This class is responsibSimulatorHandlerle for sending simulator an appropriate plans and updates
     */

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
        final Map<Integer, Float> departures = reader.getDepartures();
        // final int size = vehicles.size();
        final int size;
        if(!Configurator.getParamBool("highway.dashboard.sumoSimulation",true))
        {
            size = Configurator.getParamInt("highway.dashboard.numberOfCarsInSimulation", vehicles.size());
        }
        else
        {
            size = vehicles.size();
        }
        final int simulatorCount = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).size();
        final HighwayStorage storage = highwayEnvironment.getStorage();
        // Divide vehicles evenly to the simulators
         //
          ConnectCallback col = new ConnectCallback() {
            private int section = 1;

            @Override
            public void invoke(ProtobufFactory factory) {
                Iterator<Integer> vehicleIt = vehicles.iterator();
                PlansOut plans = new PlansOut();
             //   RadarData update = ;
                Map<Integer, Agent> agents = storage.getAgents();
                Set<Integer> plannedVehicles = new HashSet<Integer>();
                int sizeL = size;
                if(size > vehicles.size()) sizeL = vehicles.size();
                // Iterate over all configured vehicles

                for (int i = 0; i < sizeL; i++) {
                    int vehicleID = vehicleIt.next();
                    if(Configurator.getParamBool("highway.dashboard.sumoSimulation",true))
                    {
                        storage.addForInsert(vehicleID,departures.get(vehicleID));
                    }
                    else {
                        storage.addForInsert(vehicleID);
                    }
                    if (factory!= null && i < section * size / simulatorCount && i >= (section - 1) * size / simulatorCount) {
                        plannedVehicles.add(vehicleID);
                    } else {
                        plannedVehicles.add(vehicleID);
                    }
                }

                if(factory != null)
                {
                    // Create new simulator handler
                    simulatorHandlers.add(new ProtobufSimulationHandler(highwayEnvironment,plannedVehicles,factory));
                    // This is the last simulator, start the simulation
                    if (section >= simulatorCount) {
                        synchronized (simulation) {
                            simulation.notify();
                        }
                    }
                    // Increase the section so the next simulator will simulate different vehicles
                    section++;
                }
                else
                {
                    simulatorHandlers.add(new LocalSimulatorHandler(highwayEnvironment,new HashSet<Integer>(plannedVehicles)));
                }
                storage.updateCars(new RadarData());
            }
        };
        if(simulatorCount == 0) col.invoke(null);
        communicator.registerConnectCallback(col);
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

    long timeDifference;
    int numberOfPlanCalculations = 0;

    @Override
    public void handleEvent(Event event) {
        if (event.isType(SimulationEventType.SIMULATION_STARTED)) {
            System.out.println("Caught SIMULATION_STARTED from DashBoard");
        } else if (event.isType(HighwayEventType.NEW_PLAN)) {
            List<Action> actions = (List<Action>) event.getContent();
            int id = actions.get(0).getCarId();
            if (!highwayEnvironment.getStorage().getPosCurr().containsKey(id)) return;
            for (SimulatorHandler handler : simulatorHandlers) {
                if (handler.hasVehicle(id)) {
                    handler.addActions(id, actions);
                }
                if (handler.isReady()) {
                    double fTimeStamp = actions.get(0).getTimeStamp();highwayEnvironment.getStorage().getExperimentsData().calcPlanCalculation(System.currentTimeMillis());
                    numberOfPlanCalculations++;
                    handler.sendPlans(highwayEnvironment.getStorage().getPosCurr());

                }
            }
        } else if (event.isType(HighwayEventType.UPDATED)) {
              timeDifference = System.currentTimeMillis();
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
