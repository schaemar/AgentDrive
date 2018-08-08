package cz.agents.agentdrive.highway.environment;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.environment.eventbased.EventBasedEnvironment;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.SimulatorHandler;
import cz.agents.agentdrive.highway.environment.roadnet.RoadNetworkRouter;
import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.storage.HighwayEventType;
import cz.agents.agentdrive.highway.storage.HighwayStorage;
import org.apache.log4j.Logger;

import java.util.LinkedList;
import java.util.List;

/**
 * public class {@link HighwayEnvironment} provides {@link HighwayStorage},
 * {@link RoadNetwork}
 */
@SuppressWarnings("JavadocReference")
public class HighwayEnvironment {

    private final Logger logger = Logger.getLogger(HighwayEnvironment.class);

    private HighwayStorage storage;
    private RoadNetwork roadNetwork;
    private List<SimulatorHandler> simulatorHandlers = new LinkedList<SimulatorHandler>();
    private EventProcessor ep;
    private long time;
    int numberOfPlanCalculations = 0;
    long timeDifference;

    public HighwayEnvironment(final EventProcessor eventProcessor) {
//        super(eventProcessor);
//        RandomProvider.init(this);
        // Initialize Network from given xml
        ep = eventProcessor;
        XMLReader xmlReader = new XMLReader();
        roadNetwork = xmlReader.parseNetwork(Configurator.getParamString("simulator.net.folder", "nets/junction-big/"));
        RoadNetworkRouter.setRoadNet(roadNetwork);
        storage = new HighwayStorage(this);
        logger.info("Initialized storage and RoadNetwork");
    }

    public HighwayEnvironment(final EventProcessor eventProcessor, RoadNetwork roadNetwork) {
//        super(eventProcessor);
//        RandomProvider.init(this);
        ep = eventProcessor;
        this.roadNetwork = roadNetwork;
        RoadNetworkRouter.setRoadNet(roadNetwork);
        storage = new HighwayStorage(this);
        logger.info("Initialized storage and RoadNetwork");
    }

    public HighwayEnvironment(Integer time, RoadNetwork roadNetwork) {
        this.time = time;
        this.roadNetwork = roadNetwork;
        RoadNetworkRouter.setRoadNet(roadNetwork);
        storage = new HighwayStorage(this);
        logger.info("Initialized storage and RoadNetwork");
    }

    public RoadNetwork getRoadNetwork() {
        return roadNetwork;
    }


    public HighwayStorage getStorage() {
        return storage;
    }

    public void addSimulatorHandler(SimulatorHandler sim) {
        simulatorHandlers.add(sim);
    }

    public List<SimulatorHandler> getSimulatorHandlers() {
        return simulatorHandlers;
    }

    public long getCurrentTime() {
        return ep == null ? time : ep.getCurrentTime();
    }
    public void setCurrentTime(long time){
        this.time = time;
    }
}