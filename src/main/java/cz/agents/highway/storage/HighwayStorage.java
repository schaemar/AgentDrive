package cz.agents.highway.storage;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.common.event.EventProcessorEventType;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.environment.eventbased.EventBasedStorage;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.highway.agent.*;
import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.util.FileUtil;
import org.apache.log4j.Logger;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.region.MovingCircle;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;
import java.util.*;

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
    private Queue<Pair<Integer,Float>> vehiclesForInsert;
    private final float CHECKING_DISTANCE = 500;
    private final float SAFETY_RESERVE = 12;
    Comparator<Pair<Integer,Float>> comparator;

    public HighwayStorage(HighwayEnvironment environment) {
        super(environment);
        environment.getEventProcessor().addEventHandler(this);
        roadDescription = new RoadDescription(environment.getRoadNetwork());
        comparator = new QueueComparator();
        vehiclesForInsert = new PriorityQueue<Pair<Integer, Float>>(20,comparator);
        // number 20 is random, it is only needed to be java 1.7 compatible
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
        } else if (event.isType(EventProcessorEventType.STOP)) {
            logger.info("Number of collisions is " + calculateNumberOfCollisions() / 2 + "\n");
            FileUtil.getInstance().writeDistancesToFile(distances);
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
     //   if (!object.getCars().isEmpty()) {
          /*  if (object.getCars().size() == 1) {
                logger.info("Number of collisions is " + calculateNumberOfCollisions() / 2 + "\n");
                FileUtil.getInstance().writeDistancesToFile(distances);
                getEventProcessor().addEvent(EventProcessorEventType.STOP, null, null, null);
                System.out.println("Sedim na kameni a cekam");
            }*/
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
        recreate(object);
        getEventProcessor().addEvent(HighwayEventType.UPDATED, null, null, null);
     //   }
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
    public void addForInsert(int id)
    {
        vehiclesForInsert.add(new Pair<Integer, Float>(id, 0f));
    }
    public void addForInsert(int id,float time)
    {
        vehiclesForInsert.add(new Pair<Integer, Float>(id,time));
    }
    public void recreate(RadarData object) {
        Queue<Pair<Integer,Float>> notInsertedVehicles = new PriorityQueue<Pair<Integer, Float>>(20,comparator);
        while(vehiclesForInsert.peek() != null)
        {
            Pair<Integer,Float> vehicle = vehiclesForInsert.poll();
            int id = vehicle.getKey();
            if(posCurr.containsKey(id))
            {
                posCurr.remove(id);
            }
            if(agents.containsKey(id) && Configurator.getParamBool("highway.dashboard.sumoSimulation",true)) continue;
            if(isDeleted(object,id) == false)
            {
                notInsertedVehicles.add(vehicle);
                continue;
            }
            double updateTime = 0d;
            double randomUpdateTime = 0d;
           /* if(!posCurr.isEmpty()) {
                randomUpdateTime = posCurr.entrySet().iterator().next().getValue().getUpdateTime();
            }*/
            updateTime = getEventProcessor().getCurrentTime();
            if(vehicle.getValue() > updateTime/1000 ||
                    (posCurr.size() >= Configurator.getParamInt("highway.dashboard.numberOfCarsInSimulation", agents.size())))
            {
                notInsertedVehicles.add(vehicle);
                continue;
            }
            RouteNavigator routeNavigator = new RouteNavigator(id);
            Point2f position = routeNavigator.next();
            Point3f initialPosition = new Point3f(position.x, position.y, 0);
            Point2f next = routeNavigator.nextWithReset();
            Vector3f initialVelocity = new Vector3f(next.x - position.x, next.y - position.y, 0);

            int numberOftryes = 10;
            int prom = 0;

            while (!isSafe(id,initialPosition,routeNavigator) && prom < numberOftryes)
            {

                for(int i=0 ; i < 6 ;i++)
                {
                    position = routeNavigator.next();
                }
                position = routeNavigator.next();
                initialPosition.setX(position.x);
                initialPosition.setY(position.y);
                prom++;
            }

            if (prom < 10) {
                Agent agent;
                if (agents.containsKey(id)) {
                    agent = agents.get(id);
                } else {
                    agent = createAgent(id);
                }
                agent.setNavigator(routeNavigator);
                RoadObject newRoadObject = new RoadObject(id,updateTime, agent.getNavigator().getLane().getIndex(),initialPosition,initialVelocity);
                agent.getNavigator().setMyLifeEnds(false);
                updateCar(newRoadObject);
            } else
                notInsertedVehicles.add(vehicle);
        }
        while(notInsertedVehicles.peek() != null)
        {
            vehiclesForInsert.add(notInsertedVehicles.poll());
        }
    }
    private boolean isDeleted(RadarData object,int id)
    {
        for (RoadObject car : object.getCars()) {
            if(car.getId() == id)
                return false;
        }
        return true;
    }
    public boolean isSafe(int stateId,Point3f statePosition,RouteNavigator stateNavigator)
    {

        for (Map.Entry<Integer, RoadObject> obj : posCurr.entrySet()) {
            RoadObject entry = obj.getValue();
            float distanceToSecondCar = entry.getPosition().distance(statePosition);
            if(distanceToSecondCar < CHECKING_DISTANCE)
            {
                if (distanceToSecondCar < SAFETY_RESERVE) return false;
                List<Edge> followingEdgesInPlan = agents.get(entry.getId()).getNavigator().getFollowingEdgesInPlan();
                for (Edge e : followingEdgesInPlan) {
                    if (stateNavigator.getLane().equals(e)) {
                        if(agents.get(entry.getId()).getNavigator().getActualPointer() < stateNavigator.getActualPointer()) {
                            double safedist = safeDistance(-4, entry.getVelocity().length(), 0);
                            if (safedist + SAFETY_RESERVE < distanceToSecondCar) return false;
                        }
                    }
                }
            }

        }
        return true;
    }
    private double safeDistance(double a0, double v0, double v1) {
        double safeDist = (v1 * v1 - v0 * v0) / (2 * a0);
        return safeDist;
    }
    private class QueueComparator implements Comparator<Pair<Integer,Float>>
    {
        @Override
        public int compare(Pair<Integer, Float> o1, Pair<Integer, Float> o2) {
            if(o1.getValue() < o2.getValue())
            {
                return -1;
            }
            if(o1.getValue() > o2.getValue())
            {
                return 1;
            }
            return 0;
        }
    }
}
