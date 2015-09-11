package cz.agents.highway.creator;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.creator.Creator;
import cz.agents.alite.protobuf.communicator.Communicator;
import cz.agents.alite.protobuf.communicator.ServerCommunicator;
import cz.agents.alite.protobuf.communicator.callback.ConnectCallback;
import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.alite.protobuf.factory.ProtobufFactory;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.alite.transport.SocketTransportLayer;
import cz.agents.alite.transport.TransportLayerInterface;
import cz.agents.highway.agent.Agent;
import cz.agents.highway.environment.SimulatorHandlers.LocalSimulatorHandler;
import cz.agents.highway.environment.SimulatorHandlers.ProtobufSimulatorHandler;
import cz.agents.highway.environment.SimulatorHandlers.SimulatorHandler;
import cz.agents.highway.environment.roadnet.XMLReader;
import cz.agents.highway.protobuf.factory.dlr.DLR_PlansFactory;
import cz.agents.highway.protobuf.factory.dlr.DLR_UpdateFactory;
import cz.agents.highway.protobuf.factory.simplan.PlansFactory;
import cz.agents.highway.protobuf.factory.simplan.UpdateFactory;
import cz.agents.highway.protobuf.generated.dlr.DLR_MessageContainer;
import cz.agents.highway.protobuf.generated.simplan.MessageContainer;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.HighwayStorage;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.PlansOut;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import java.io.IOException;
import java.net.URI;
import java.util.*;

/**
 * Dash board like controller, that manages launching simulators and their synchronization with agentDrive
 * <p/>
 * Created by wmatex on 27.6.14.
 */
public class DashBoardController extends DefaultCreator implements EventHandler, Creator {
    private Communicator communicator;
    private final Logger logger = Logger.getLogger(DashBoardController.class);
    boolean meas = false;  //debug
    long time   = 0;       //debug
    protected long timestep = Configurator.getParamInt("highway.timestep", 100);
    boolean isProtobufOn = Configurator.getParamBool("highway.protobuf.isOn", false);

    /**
     * This class is responsibSimulatorHandlerle for sending simulator an appropriate plans and updates
     */

    /// Map of all running simulator processes
    private Map<String, Process> simulators = new HashMap<String, Process>();

    @Override
    public void init(String[] args) {
        super.init(args);
        logger.setLevel(Level.INFO);
    }

    @Override
    public void create() {
        super.create();
        if(Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).isEmpty()) {
            isProtobufOn = false;
        }
        else {
            for (String s : (Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class))) {
                if (s.equals("SimulatorLite")) {
                    initProtoCommunicator();
                    isProtobufOn = true;
                    break;
                }
            }
        }
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
        if (!Configurator.getParamBool("highway.dashboard.sumoSimulation", true)) {
            size = Configurator.getParamInt("highway.dashboard.numberOfCarsInSimulation", vehicles.size());
        } else {
            size = vehicles.size();
        }
        final int simulatorCount = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).size();
        final HighwayStorage storage = highwayEnvironment.getStorage();
        // Divide vehicles evenly to the simulators
        //
        Iterator<Integer> vehicleIt = vehicles.iterator();
        PlansOut plans = new PlansOut();
        //   RadarData update = ;
        Map<Integer, Agent> agents = storage.getAgents();
        Set<Integer> plannedVehiclesLocal = new HashSet<Integer>();
        int sizeL = size;
        if (size > vehicles.size()) sizeL = vehicles.size();
        // Iterate over all configured vehicles

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


        ConnectCallback col = new ConnectCallback() {
            private int section = 1;
            @Override
            public void invoke(ProtobufFactory factory) {
                // Create new simulator handler
                highwayEnvironment.addSimulatorHandler(new ProtobufSimulatorHandler(highwayEnvironment, plannedVehicles, factory));
                // This is the last simulator, start the simulation
                if (section >= simulatorCount) {
                    synchronized (simulation) {
                        simulation.notify();
                    }
                }
                // Increase the section so the next simulator will simulate different vehicles
                section++;
                storage.updateCars(new RadarData());
            }
        };
        if (Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).isEmpty())
        { //Simulator dependent code.
            highwayEnvironment.addSimulatorHandler(new LocalSimulatorHandler(highwayEnvironment, new HashSet<Integer>(plannedVehicles)));
            storage.updateCars(new RadarData());
        }
        if(isProtobufOn) communicator.registerConnectCallback(col);
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
            getEventProcessor().addEvent(HighwayEventType.TIMESTEP, null, null, null, timestep);
        }
        else if (event.isType(HighwayEventType.TIMESTEP)) {
            if(isProtobufOn) communicator.run();
            getEventProcessor().addEvent(HighwayEventType.TIMESTEP, null, null, null, timestep);
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
    private void initProtoCommunicator() {
        FactoryInterface factoryUpdate = null;
        FactoryInterface factoryPlans = null;
        TransportLayerInterface transportInterface = new SocketTransportLayer();
        String uri = Configurator.getParamString("highway.protobuf.uri",
                "socket://localhost:2222");

        // initializing protobuf communicator sending by a thread, but not receiveing by thread
        // (cannot addEvent to EventQUeue from different threads)
        boolean isSendThread = true;
        boolean isReceiveThread = false;
        int port = URI.create(uri).getPort();

        String protocol = Configurator.getParamString("highway.protobuf.protocol", "DLR");
        if (protocol.equals("DLR")) {
            communicator = new ServerCommunicator<DLR_MessageContainer.Header, DLR_MessageContainer.Message>(
                    port, DLR_MessageContainer.Header.getDefaultInstance(),
                    DLR_MessageContainer.Message.getDefaultInstance(), transportInterface, isSendThread, isReceiveThread);
            factoryUpdate = new DLR_UpdateFactory();
            factoryPlans = new DLR_PlansFactory();

        } else if (protocol.equals("simplan")) {
            communicator = new ServerCommunicator<MessageContainer.Header, MessageContainer.Message>(
                    port, MessageContainer.Header.getDefaultInstance(), MessageContainer.Message.getDefaultInstance(),
                    transportInterface, isSendThread, isReceiveThread
            );
            factoryUpdate = new UpdateFactory();
            factoryPlans = new PlansFactory();
        }

        try {
            communicator.registerOutFactory(factoryPlans);
            communicator.registerOutFactory(factoryUpdate);
            communicator.registerReceiveCallback(factoryUpdate, new ProtobufFactory.ProtobufMessageHandler<RadarData>() {

                public void notify(RadarData object) {
                    if (object != null) {
                        meas = true;
                        time = System.currentTimeMillis();

                        logger.debug("Received RadarData");
                        highwayEnvironment.getStorage().updateCars(object);

                    }
                }
            });
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
