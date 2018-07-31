package cz.agents.agentdrive.simulator.lite.environment;

import cz.agents.agentdrive.highway.agent.Agent;
import cz.agents.agentdrive.highway.environment.HighwayEnvironment;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.ModuleSimulatorHandler;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.PlanCallback;
import cz.agents.agentdrive.highway.environment.roadnet.RoadNetworkRouter;
import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.storage.HighwayEventType;
import cz.agents.agentdrive.highway.storage.HighwayStorage;
import cz.agents.agentdrive.highway.storage.RadarData;
import cz.agents.agentdrive.highway.storage.RoadObject;
import cz.agents.agentdrive.highway.storage.plan.Action;
import cz.agents.agentdrive.highway.storage.plan.PlansOut;
import cz.agents.agentdrive.highway.storage.plan.WPAction;
import cz.agents.agentdrive.simulator.lite.storage.SimulatorEvent;
import cz.agents.agentdrive.simulator.lite.storage.VehicleStorage;
import cz.agents.agentdrive.simulator.lite.storage.vehicle.Car;
import cz.agents.agentdrive.simulator.lite.storage.vehicle.Vehicle;
import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventHandler;
import cz.agents.alite.common.event.EventProcessor;

import cz.agents.alite.communication.Communicator;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.environment.eventbased.EventBasedEnvironment;
import cz.agents.alite.simulation.SimulationEventType;

import org.apache.log4j.Logger;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;
import java.io.IOException;
import java.util.*;

/**
 * Class representing the environment of the simulation
 * <p/>
 * Created by wmatex on 3.7.14.
 */
public class SimulatorEnvironment extends EventBasedEnvironment {
    private static final Logger logger = Logger.getLogger(SimulatorEnvironment.class);
    /// All simulated vehicles
    private VehicleStorage vehicleStorage;
    private HighwayEnvironment highwayEnvironment;
    private RoadNetwork roadNetwork;
    private RadarData currentState = new RadarData();
    private PlansOut plansOut = new PlansOut();
    private PlanCallback planCallback = new PlanCallbackImp();
    protected long timestep = Configurator.getParamInt("simulator.lite.timestep", 100);
    private int counter = 0;


    /// Time between updates
    static public final long UPDATE_STEP = 20;

    /// Time between communication updates
    static public final long COMM_STEP = 200;

    /**
     * Create the environment, the storage and register event handlers
     */
    public SimulatorEnvironment(final EventProcessor eventProcessor) {
        super(eventProcessor);

        //initTraffic();

        vehicleStorage = new VehicleStorage(this);
        XMLReader xmlReader = new XMLReader();
        roadNetwork = xmlReader.parseNetwork(Configurator.getParamString("simulator.net.folder", "nets/junction-big/"));
        RoadNetworkRouter.setRoadNet(roadNetwork);
        highwayEnvironment = new HighwayEnvironment(this.getEventProcessor(), roadNetwork);
        getEventProcessor().addEventHandler(new EventHandler() {
            @Override
            public EventProcessor getEventProcessor() {
                return eventProcessor;
            }

            @Override
            public void handleEvent(Event event) {
                // Start updating vehicles when the simulation starts
                if (event.isType(SimulationEventType.SIMULATION_STARTED)) {
                    logger.info("SIMULATION STARTED");
                    //highwayEnvironment.getStorage().updateCars(vehicleStorage.generateRadarData());

                    getEventProcessor().addEvent(HighwayEventType.TIMESTEP, null, null, null);
                    getEventProcessor().addEvent(SimulatorEvent.UPDATE, null, null, null);
                    getEventProcessor().addEvent(SimulatorEvent.COMMUNICATION_UPDATE, null, null, null);
                } else if (event.isType(SimulatorEvent.COMMUNICATION_UPDATE)) {
                    //highwayEnvironment.getStorage().updateCars(vehicleStorage.generateRadarData());
                    //highwayEnvironment.getStorage().updateCars(currentState);
//                    if (counter > 0)highwayEnvironment.getStorage().updateCars(vehicleStorage.generateRadarData());
//                    getEventProcessor().addEvent(SimulatorEvent.COMMUNICATION_UPDATE, null, null, null, Math.max(1, (long) (timestep * 50)));
                } else if (event.isType(HighwayEventType.TIMESTEP)) {
                    getEventProcessor().addEvent(HighwayEventType.TIMESTEP, null, null, null, timestep);
                }
            }
        });
    }

    private void acceptPlans(PlansOut plans) {
        logger.debug("Received plans: " + plans.getCarIds());
        for (int carID : plans.getCarIds()) {
            logger.debug("Plan for car " + carID + ": " + plans.getPlan(carID));

            // This is the init plan
            if (((WPAction) plans.getPlan(carID).iterator().next()).getSpeed() == -1) {
                LinkedList<Vehicle> allVehicles = vehicleStorage.getAllVehicles();
                vehicleStorage.removeVehicle(carID);
            } else {
                Vehicle vehicle = vehicleStorage.getVehicle(carID);
                if (vehicle == null) {

                    // Get the first action containing car info
                    WPAction action = (WPAction) plans.getPlan(carID).iterator().next();
                    if (plans.getPlan(carID).size() > 1) {
                        Iterator<Action> iter = plans.getPlan(carID).iterator();
                        Point3f first = ((WPAction) iter.next()).getPosition();
                        Point3f second = ((WPAction) iter.next()).getPosition();
                        Vector3f heading = new Vector3f(second.x - first.x, second.y - first.y, second.z - first.z);
                        heading.normalize();

                        vehicleStorage.addVehicle(new Car(carID, 0, action.getPosition(), heading, (float) action.getSpeed() /*30.0f*/));
                    } else {

                        vehicleStorage.addVehicle(new Car(carID, 0, action.getPosition(), plans.getCurrStates().get(carID).getVelocity(), (float) action.getSpeed()));
                    }
                } else {
                    vehicle.getVelocityController().updatePlan(plans.getPlan(carID));
                    vehicle.setWayPoints(plans.getPlan(carID));
                }
            }
        }
    }

    /**
     * Initialize the environment and add some vehicles
     */
    public void init() {
        getEventProcessor().addEventHandler(vehicleStorage);
        // getEventProcessor().addEventHandler(highwayEnvironment);
    }

    public VehicleStorage getStorage() {
        return vehicleStorage;
    }

    public RoadNetwork getRoadNetwork() {
        return roadNetwork;
    }

    public HighwayEnvironment getHighwayEnvironment() {
        return highwayEnvironment;
    }

    public PlanCallback getPlanCallback() {
        return planCallback;
    }

    class PlanCallbackImp extends PlanCallback {
        //final HashSet<Integer> plannedVehicles = new HashSet<Integer>();
        @Override
        public void execute(PlansOut plans) {

            Map<Integer, RoadObject> currStates = plans.getCurrStates();
            RadarData radarData = new RadarData();
            float duration = 0;
            float lastDuration = 0;
            double timest = Configurator.getParamDouble("highway.SimulatorLocal.timestep", 1.0);
            float timestep = (float) timest;

            boolean removeCar = false;
            for (Integer carID : plans.getCarIds()) {
                Collection<Action> plan = plans.getPlan(carID);
                RoadObject state = currStates.get(carID);
                Point3f lastPosition = state.getPosition();
                Point3f myPosition = state.getPosition();
                for (Action action : plan) {
                    if (action.getClass().equals(WPAction.class)) {
                        WPAction wpAction = (WPAction) action;
                        if (wpAction.getSpeed() == -1) {
                            myPosition = wpAction.getPosition();
                            removeCar = true;
                            System.out.println("removed - speed = " + wpAction.getSpeed());
                        }
                        if (wpAction.getSpeed() < 0.001) {
                            duration += 0.10f;
                        } else {
                            myPosition = wpAction.getPosition();
                            lastDuration = (float) (wpAction.getPosition().distance(lastPosition) / (wpAction.getSpeed()));
                            duration += wpAction.getPosition().distance(lastPosition) / (wpAction.getSpeed());
                        }
                        // creating point between the waypoints if my duration is greater than the defined timestep
                        if (duration >= timestep) {

                            float remainingDuration = timestep - (duration - lastDuration);
                            float ration = remainingDuration / lastDuration;
                            float x = myPosition.x - lastPosition.x;
                            float y = myPosition.y - lastPosition.y;
                            float z = myPosition.z - lastPosition.z;
                            Vector3f vec = new Vector3f(x, y, z);
                            vec.scale(ration);

                            myPosition = new Point3f(vec.x + 0 + lastPosition.x, vec.y + lastPosition.y, vec.z + lastPosition.z);
                            break;
                        }
                        lastPosition = wpAction.getPosition();
                    }
                }
                if (removeCar) {
                    if (Configurator.getParamBool("highway.dashboard.sumoSimulation", true)) {
                        this.addToPlannedVehiclesToRemove(carID);
                    }
                    removeCar = false;
                } else {
                    Vector3f vel = new Vector3f(state.getPosition());
                    vel.negate();
                    vel.add(myPosition);
                    if (vel.length() < 0.0001) {
                        vel = state.getVelocity();
                        vel.normalize();
                        vel.scale(0.0010f);
                    }
                    int lane = highwayEnvironment.getRoadNetwork().getClosestLane(myPosition).getIndex();
                    state = new RoadObject(carID, highwayEnvironment.getEventProcessor().getCurrentTime(), lane, myPosition, vel);
                    radarData.add(state);
                    duration = 0;
                }
            }
          //    acceptPlans(plans);
            plansOut = plans;
            //radarData = vehicleStorage.generateRadarData();
            //currentState = radarData;

            counter++;
            highwayEnvironment.getEventProcessor().addEvent(HighwayEventType.RADAR_DATA, highwayEnvironment.getStorage(), null, radarData, Math.max(1, (long) (timestep * 1000)));
        }
    }


}
