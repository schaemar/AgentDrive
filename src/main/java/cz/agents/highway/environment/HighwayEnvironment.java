package cz.agents.highway.environment;

import java.io.IOException;
import java.net.URI;
import java.util.LinkedList;
import java.util.List;

import javax.vecmath.Vector3d;

import cz.agents.alite.protobuf.communicator.Communicator;
import cz.agents.alite.protobuf.communicator.ServerCommunicator;
import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.alite.transport.SocketTransportLayer;
import cz.agents.alite.transport.TransportLayerInterface;
import cz.agents.highway.environment.SimulatorHandlers.SimulatorHandler;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.environment.roadnet.XMLReader;
import cz.agents.highway.protobuf.factory.simplan.PlansFactory;
import cz.agents.highway.protobuf.factory.simplan.UpdateFactory;
import cz.agents.highway.protobuf.generated.dlr.DLR_MessageContainer;
import cz.agents.highway.protobuf.generated.simplan.MessageContainer;
import org.apache.log4j.Logger;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.environment.Action;
import cz.agents.alite.environment.Sensor;
import cz.agents.alite.environment.eventbased.EventBasedEnvironment;
import cz.agents.alite.protobuf.factory.ProtobufFactory.ProtobufMessageHandler;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.highway.protobuf.factory.dlr.DLR_PlansFactory;
import cz.agents.highway.protobuf.factory.dlr.DLR_UpdateFactory;
import cz.agents.highway.protobuf.generated.dlr.DLR_MessageContainer.Header;
import cz.agents.highway.protobuf.generated.dlr.DLR_MessageContainer.Message;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.HighwayStorage;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.plan.PlansOut;

/**
 * public class {@link HighwayEnvironment} provides {@link CarStorage}, {@link HighwayStorage},
 * {@link HighwayEnvironmentHandler}
 */
@SuppressWarnings("JavadocReference")
public class HighwayEnvironment extends EventBasedEnvironment implements EventHandler {

    private final Logger logger = Logger.getLogger(HighwayEnvironment.class);
    
    private final static double ENVIRONMENT_WIDTH  = 10000.0;
    private final static double ENVIRONMENT_HEIGHT = 10000.0;
    private final static double ENVIRONMENT_DEPTH  = 15000.0;
 //   private final HighwayEnvironmentHandler handler;

    private HighwayStorage storage;
    private Network roadNetwork;
    int numberOfPlanCalculations = 0;

    private List<SimulatorHandler> simulatorHandlers = new LinkedList<SimulatorHandler>();


    // [DEBUG]
    int counter = 0;
    long min    = 1000000000;
    long max    = -1;
    double sum    = 0;
    double hundrt = 0;
    //--------

    public HighwayEnvironment(final EventProcessor eventProcessor) {
        super(eventProcessor);
        RandomProvider.init(this);


        final boolean isProtobufOn = Configurator.getParamBool("highway.protobuf.isOn", false);

     //   handler = new HighwayEnvironmentHandler();

        // Initialize Network from given xml
        XMLReader.getInstance().read(Configurator.getParamString("highway.net.folder","nets/junction-big/"));
        roadNetwork = Network.getInstance();

        storage = new HighwayStorage(this);
        logger.info("Initialized handler and storages");

        final PlansOut plans = new PlansOut();
        eventProcessor.addEventHandler(this);


        eventProcessor.addEventHandler(new EventHandler() {
            public void handleEvent(Event event) {


            }

            public EventProcessor getEventProcessor() {
                return eventProcessor;
            }

        });

      

    }

    public Network getRoadNetwork() {
        return roadNetwork;
    }

    /**
     * public class TacticalEnvironmentHandler provides methods addAction, addSensor and
     * getEnvironmentDimensions
     */
    /*
    @Override
    public HighwayEnvironmentHandler handler() {
        return handler;
    }
    public class HighwayEnvironmentHandler extends EventBasedHandler {

        protected HighwayEnvironmentHandler() {
        }

        @Override
        public <C extends Action> C addAction(Class<C> clazz, Entity entity) {
            return instantiateEnvironmentClass(clazz, entity, new Class<?>[] {
                    HighwayEnvironment.class, Entity.class });
        }

        @Override
        public <C extends Sensor> C addSensor(Class<C> clazz, Entity entity) {
            return instantiateEnvironmentClass(clazz, entity, new Class<?>[] {
                    HighwayEnvironment.class, Entity.class });
        }

        public Vector3d getEnvironmentDimensions() {
            return new Vector3d(ENVIRONMENT_WIDTH, ENVIRONMENT_HEIGHT, ENVIRONMENT_DEPTH);
        }

    }
    */


    public HighwayStorage getStorage() {
        return storage;
    }

    long timeDifference;
    @Override
    public void handleEvent(Event event) {
         if (event.isType(HighwayEventType.NEW_PLAN)) {
             List<cz.agents.highway.storage.plan.Action> actions = (List<cz.agents.highway.storage.plan.Action>) event.getContent();
             int id = actions.get(0).getCarId();
             if (!getStorage().getPosCurr().containsKey(id)) return;
             for (SimulatorHandler handler : simulatorHandlers) {
                 if (handler.hasVehicle(id)) {
                     handler.addActions(id, actions);
                 }
                 if (handler.isReady()) {
                     double fTimeStamp = actions.get(0).getTimeStamp();
                     getStorage().getExperimentsData().calcPlanCalculation(System.currentTimeMillis());
                     numberOfPlanCalculations++;
                     handler.sendPlans(getStorage().getPosCurr());
                 }
             }
         }
         else if (event.isType(HighwayEventType.UPDATED)) {
             timeDifference = System.currentTimeMillis();
         }
    }

    public void addSimulatorHandler(SimulatorHandler sim)
    {
        simulatorHandlers.add(sim);
    }
}