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
public class HighwayEnvironment extends EventBasedEnvironment {

    private final Logger logger = Logger.getLogger(HighwayEnvironment.class);

    private HighwayStorage storage;
    private RoadNetwork roadNetwork;
    private List<SimulatorHandler> simulatorHandlers = new LinkedList<SimulatorHandler>();

    int numberOfPlanCalculations = 0;
    long timeDifference;

    public HighwayEnvironment(final EventProcessor eventProcessor) {
        super(eventProcessor);
        RandomProvider.init(this);

        // Initialize Network from given xml
        XMLReader xmlReader = new XMLReader();
        roadNetwork = xmlReader.parseNetwork(Configurator.getParamString("simulator.net.folder", "nets/junction-big/"));
        RoadNetworkRouter.setRoadNet(roadNetwork);
        storage = new HighwayStorage(this);
        logger.info("Initialized storage and RoadNetwork");

        // eventProcessor.addEventHandler(this);
    }

    public HighwayEnvironment(final EventProcessor eventProcessor, RoadNetwork roadNetwork) {
        super(eventProcessor);
        RandomProvider.init(this);

        // Initialize Network from given xml
        XMLReader xmlReader = new XMLReader();
        this.roadNetwork = roadNetwork;//xmlReader.parseNetwork(Configurator.getParamString("simulator.net.folder", "nets/junction-big/"));
        RoadNetworkRouter.setRoadNet(roadNetwork);
        storage = new HighwayStorage(this);
        logger.info("Initialized storage and RoadNetwork");

        //  eventProcessor.addEventHandler(this);
    }

    public RoadNetwork getRoadNetwork() {
        return roadNetwork;
    }


    public HighwayStorage getStorage() {
        return storage;
    }


    public void handleEvent(Event event) {
        if (event.isType(HighwayEventType.NEW_PLAN)) {
//            List<cz.agents.agentdrive.highway.storage.plan.Action> actions = (List<cz.agents.agentdrive.highway.storage.plan.Action>) event.getContent();
//            int id = actions.get(0).getCarId();
//            if (!getStorage().getPosCurr().containsKey(id)) return;
//            for (SimulatorHandler handler : simulatorHandlers) {
//                if (handler.hasVehicle(id)) {
//                    handler.addActions(id, actions);
//                }
//                if (handler.isReady()) {
//                    getStorage().getExperimentsData().calcPlanCalculation(System.currentTimeMillis());
//                    numberOfPlanCalculations++;
//                    handler.sendPlans(getStorage().getPosCurr());
//                }
//            }
        } else if (event.isType(HighwayEventType.UPDATED)) {
            //timeDifference = System.currentTimeMillis();
        }
    }

    public void addSimulatorHandler(SimulatorHandler sim) {
        simulatorHandlers.add(sim);
    }

    public List<SimulatorHandler> getSimulatorHandlers() {
        return simulatorHandlers;
    }
}