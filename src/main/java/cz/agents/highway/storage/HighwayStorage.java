package cz.agents.highway.storage;

import java.util.*;

import cz.agents.alite.common.event.EventProcessorEventType;
import cz.agents.alite.configurator.Configurator;
import cz.agents.highway.agent.*;
import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.environment.roadnet.XMLReader;
import cz.agents.highway.util.FileUtil;
import org.apache.log4j.Logger;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.environment.eventbased.EventBasedStorage;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.highway.storage.plan.Action;
import tt.euclid2i.Trajectory;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;

public class HighwayStorage extends EventBasedStorage {

    private final Logger logger = Logger.getLogger(HighwayStorage.class);

    private final RoadDescription roadDescription;
    private final Map<Integer, Agent> agents = new LinkedHashMap<Integer, Agent>();
    private final Map<Integer, RoadObject> posCurr = new LinkedHashMap<Integer, RoadObject>();
    private final Map<Integer, Action> actions = new LinkedHashMap<Integer, Action>();
    private Map<Integer, Queue<Float>> distances = new LinkedHashMap<Integer, Queue<Float>>();
    private final float SAVE_DISTANCE = 10;
    Point3f refcar = new Point3f(0, 0, 0);
    private final Map<Integer, Region> trajectories = new LinkedHashMap<Integer, Region>();

    private Agent queen;

    public HighwayStorage(HighwayEnvironment environment) {
        super(environment);
        environment.getEventProcessor().addEventHandler(this);
        roadDescription = new RoadDescription(environment.getRoadNetwork());

    }

    @Override
    public void handleEvent(Event event) {

        if (event.isType(SimulationEventType.SIMULATION_STARTED)) {
            logger.debug("HighwayStorage: handled simulation START");
        } else if (event.isType(HighwayEventType.RADAR_DATA)) {
            logger.debug("HighwayStorage: handled: RADAR_DATA");
            RadarData radar_data = (RadarData) event.getContent();
            updateCars(radar_data);
        } else if (event.isType(HighwayEventType.TRAJECTORY_UPDATED)) {
            Map.Entry<Integer, Region> agentTrajectory = (Map.Entry<Integer, Region>) event.getContent();
            MovingCircle stored = (MovingCircle) trajectories.get(agentTrajectory.getKey());
            MovingCircle inc    = (MovingCircle) agentTrajectory.getValue();
            if (stored == null || !stored.getTrajectory().equals(inc.getTrajectory())) {
                trajectories.put(agentTrajectory.getKey(), agentTrajectory.getValue());
                logger.debug("Changed trajectory of agent: "+agentTrajectory.getKey());
                getEnvironment().getEventProcessor().addEvent(HighwayEventType.TRAJECTORY_CHANGED, null, null, agentTrajectory.getKey());
            }
        }

    }

    public void updateCar(RoadObject carState) {
        int carId = carState.getId();

//        if (!agents.containsKey(carId)) {
//            createAgent(carId);
//        }
        posCurr.put(carId, carState);

    }

    public Agent createAgent(final int id) {
        String agentClassName = Configurator.getParamString("highway.agent", "RouteAgent");
        Agent agent = null;
        if (agentClassName.equals("RouteAgent")) {
            agent = new RouteAgent(id);
        } else if (agentClassName.equals("SDAgent")) {
            agent = new SDAgent(id);
        } else if (agentClassName.equals("GSDAgent")) {
            agent = new GSDAgent(id, (HighwayEnvironment) getEnvironment());

        } else if (agentClassName.equals("ADPPAgent")) {
            agent = new ADPPAgent(id);
        }
        VehicleSensor sensor = new VehicleSensor(getEnvironment(), agent, this);
        VehicleActuator actuator = new VehicleActuator(getEnvironment(), agent, this);
        agent.addSensor(sensor);
        agent.addActuator(actuator);

        agents.put(id, agent);
        return agent;
    }

    public void act(int carId, Action action) {
        actions.put(carId, action);

    }

    public void act(int carId, List<Action> action) {
        actions.put(carId, action.get(0));

    }

    public RoadDescription getRoadDescription() {
        return roadDescription;
    }

    public Map<Integer, Agent> getAgents() {
        return agents;
    }

    public Map<Integer, RoadObject> getPosCurr() {
        return posCurr;
    }

    public Map<Integer, Action> getActions() {
        return actions;
    }

    public Map<Integer, Region> getTrajectories() {
        return trajectories;
    }
    public void updateCars(RadarData object) {
        if (!object.getCars().isEmpty()) {
            if (object.getCars().size() == 1) {
                logger.info("Number of collisions is " + calculateNumberOfCollisions() / 2 + "\n");
                FileUtil.getInstance().writeDistancesToFile(distances);
                getEventProcessor().addEvent(EventProcessorEventType.STOP, null, null, null);
                System.out.println("Sedim na kameni a cekam");
            }
            for (RoadObject car : object.getCars()) {
                updateCar(car);
            }
            logger.debug("HighwayStorage updated vehicles: received " + object);


            for (Map.Entry<Integer, RoadObject> entry : posCurr.entrySet()) {
                Queue<Float> original = distances.get(entry.getKey());
                if (original == null)
                    original = new LinkedList<Float>();
                Float dist = entry.getValue().getPosition().distance(new Point3f(0f, 0f, 0f));
                /*
                try {
                    Point3f temp = posCurr.get(4).getPosition();
                    refcar = temp;
                }
                catch(NullPointerException exp)
                {
                    //TODO Fix this structure
                }
                 Float dist = entry.getValue().getPosition().distance(refcar);
                 */
                if (original.isEmpty() || dist < original.peek()) {
                    original.add(dist);
                }
                distances.put(entry.getKey(), original);
            }
            getEventProcessor().addEvent(HighwayEventType.UPDATED, null, null, null);
        }
    }

    private int calculateNumberOfCollisions() {
        int num = 0;
        Iterator it = agents.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry pair = (Map.Entry) it.next();
            if (pair.getValue() instanceof GSDAgent) {
                num += ((GSDAgent) pair.getValue()).getNumberOfColisions();
            }
            //System.out.println(pair.getKey() + " = " + pair.getValue());
            it.remove(); // avoids a ConcurrentModificationException
        }
        return num;
    }
    public void removeAgent(Integer carID) {
        agents.remove(carID);
    }
    public void recreate(int id) {
        agents.get(id).getNavigator().hardReset();
    }
}