package cz.agents.highway.creator;

import java.io.IOException;
import java.net.URI;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.protobuf.communicator.Communicator;
import cz.agents.alite.protobuf.communicator.ServerCommunicator;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.alite.transport.SocketTransportLayer;
import cz.agents.alite.transport.TransportLayerInterface;
import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.protobuf.generated.MessageContainer;
import cz.agents.highway.protobuf.generated.dlr.DLR_MessageContainer;
import org.apache.log4j.Logger;

/**
 * Dash board like controller, that manages launching simulators and their synchronization with highway
 *
 * Created by wmatex on 27.6.14.
 */
public class DashBoardController implements EventHandler {
    private final EventProcessor eventProcessor;
    private Communicator<DLR_MessageContainer.Header, DLR_MessageContainer.Message> communicator;
    private final Logger logger = Logger.getLogger(HighwayEnvironment.class);

    /// Map of all running simulator processes
    private Map<String, Process> simulators = new HashMap<String, Process>();

    public DashBoardController(EventProcessor eventProcessor) {
        this.eventProcessor = eventProcessor;

        initCommunicator();
        eventProcessor.addEventHandler(this);
    }

    private void initCommunicator() {
        TransportLayerInterface transportInterface = new SocketTransportLayer();
        String uri = Configurator.getParamString("highway.protobuf.uri",
                "socket://localhost:2222");

        // initializing protobuf communicator sending by a thread, but not receiveing by thread
        // (cannot addEvent to EventQUeue from different threads)
        boolean isSendThread = true;
        boolean isReceiveThread = false;
        communicator = new ServerCommunicator<DLR_MessageContainer.Header, DLR_MessageContainer.Message>(
                URI.create(uri).getPort(), DLR_MessageContainer.Header.getDefaultInstance(),
                DLR_MessageContainer.Message.getDefaultInstance(), transportInterface, isSendThread, isReceiveThread);
    }

    public Communicator getCommunicator() {
        return communicator;
    }

    public void startSimulation() {
        // Start simulators
        List<String> simulatorsToRun = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class);
        logger.info("Running this simulators: "+simulatorsToRun);
        try {
            for (String sim : simulatorsToRun) {
                runSimulator(sim);
            }
            eventProcessor.run();
        } catch (IOException e) {
            logger.error("Failed to run a simulator: "+e);
            e.printStackTrace();
        }
    }

    @Override
    public EventProcessor getEventProcessor() {
        return eventProcessor;
    }

    @Override
    public void handleEvent(Event event) {
        if (event.isType(SimulationEventType.SIMULATION_STARTED)) {
            System.out.println("Caught SIMULATION_STARTED from DashBoard");
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
