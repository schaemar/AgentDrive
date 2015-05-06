package cz.agents.highway.environment;

import java.io.IOException;
import java.net.URI;

import javax.vecmath.Vector3d;

import cz.agents.alite.protobuf.communicator.Communicator;
import cz.agents.alite.protobuf.communicator.ServerCommunicator;
import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.alite.transport.SocketTransportLayer;
import cz.agents.alite.transport.TransportLayerInterface;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.environment.roadnet.XMLReader;
import cz.agents.highway.platooning.PlatooningAgent;
import cz.agents.highway.platooning.PlatooningCenterModule;
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
public class HighwayEnvironment extends EventBasedEnvironment {

    private final Logger logger = Logger.getLogger(HighwayEnvironment.class);
    
    private final static double ENVIRONMENT_WIDTH  = 10000.0;
    private final static double ENVIRONMENT_HEIGHT = 10000.0;
    private final static double ENVIRONMENT_DEPTH  = 15000.0;
    private final HighwayEnvironmentHandler handler;

    private Communicator communicator;
    private HighwayStorage storage;
    private PlatooningCenterModule platooningModule;
    private Network roadNetwork;

    // [DEBUG]
    int counter = 0;
    long time   = 0;
    long min    = 1000000000;
    long max    = -1;
    double sum    = 0;
    boolean meas = false;
    double hundrt = 0;
    //--------
    protected long timestep;

    public HighwayEnvironment(final EventProcessor eventProcessor) {
        super(eventProcessor);
        RandomProvider.init(this);

        timestep = Configurator.getParamInt("highway.timestep", 100);
        final boolean isProtobufOn = Configurator.getParamBool("highway.protobuf.isOn", false);

        handler = new HighwayEnvironmentHandler();

        // Initialize Network from given xml
        XMLReader.getInstance().read(Configurator.getParamString("highway.net.folder","nets/junction-big/"));
        roadNetwork = Network.getInstance();

        storage = new HighwayStorage(this);
        platooningModule = new PlatooningCenterModule(storage);
        logger.info("Initialized handler and storages");

        final PlansOut plans = new PlansOut();

        initProtoCommunicator();

        eventProcessor.addEventHandler(new EventHandler() {
            public void handleEvent(Event event) {
                if (event.isType(HighwayEventType.TIMESTEP)) {
                    communicator.run(); //should use this, to avoid adding events to EventProcessor from different thread
                   //long timeout = timestep - (getEventProcessor().getCurrentTime()-(lastTick+timestep));
                    getEventProcessor().addEvent(HighwayEventType.TIMESTEP, null, null, null, timestep);
                } else if (event.isType(SimulationEventType.SIMULATION_STARTED)) {
//                    if(isProtobufOn){
//                        initProtoCommunicator();
//                    }
                    getEventProcessor().addEvent(HighwayEventType.TIMESTEP, null, null, null, timestep);
//                } else if (event.isType(HighwayEventType.NEW_PLAN)) {
//                    try {
//                        plans.addAction((cz.agents.highway.storage.plan.Action) event.getContent());
//                        // send all plans at once
//                        if (plans.getCarIds().size() == getStorage().getAgents().size()) {
//                            logger.debug("Sending new plans");
//
//                            comm.send(new PlansOut(plans));
//
//                            if(!meas) logger.error("TO SE NESMI STAT");
//                            else {
//                            	meas = false;
//                            	long duration = System.currentTimeMillis() - time;
//                            	min = Math.min(min, duration);
//                            	max = Math.max(max, duration);
//                            	counter++;
//                            	sum += duration;
//                            	if(counter == 100){
//                            		System.out.println("HUNDRT: " + 10/(time - hundrt));
//                            		hundrt = time;
//                            		System.out.println("------------");
//                            		System.out.println("MIN: " + min);
//                            		System.out.println("MAX: " + max);
//                            		System.out.println("AVG: " + (sum/counter));
//                            		min = 1000000000;
//                            		max = -1;
//                            		sum = 0;
//                            		counter = 0;
//                            	}
//                            }
//                            plans.clear();
//                        }
//
//                    } catch (IOException e) {
//                        e.printStackTrace();
//                    }
                }

            }

            public EventProcessor getEventProcessor() {
                return eventProcessor;
            }

        });

      

    }

    @Override
    public HighwayEnvironmentHandler handler() {
        return handler;
    }

    public Network getRoadNetwork() {
        return roadNetwork;
    }

    /**
     * public class TacticalEnvironmentHandler provides methods addAction, addSensor and
     * getEnvironmentDimensions
     */
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
            communicator = new ServerCommunicator<Header, Message>(
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
            communicator.registerReceiveCallback(factoryUpdate, new ProtobufMessageHandler<RadarData>() {

                public void notify(RadarData object) {
                    if (object != null) {
                    	meas = true;
                    	time = System.currentTimeMillis();
                    	
                        logger.debug("Received RadarData");
                        storage.updateCars(object);
                        platooningModule.run();

                    }
                }
            });
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public HighwayStorage getStorage() {
        return storage;
    }
    public PlatooningCenterModule getPlatooningModule(){
        return platooningModule;
    }

    public Communicator getCommunicator() {
        return communicator;
    }

    public void runPlatooningModule(){
        platooningModule.run();
    }
}