package cz.agents.agentdrive.highway.agent;
import cz.agents.agentdrive.highway.environment.roadnet.*;
import java.util.*;

import javax.vecmath.*;

import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.storage.HighwayEventType;
import cz.agents.agentdrive.highway.storage.VehicleSensor;
import cz.agents.alite.common.event.Event;
import org.apache.log4j.Logger;

import cz.agents.alite.configurator.Configurator;
import cz.agents.agentdrive.highway.environment.RandomProvider;
import cz.agents.agentdrive.highway.maneuver.AccelerationManeuver;
import cz.agents.agentdrive.highway.maneuver.CarManeuver;
import cz.agents.agentdrive.highway.maneuver.DeaccelerationManeuver;
import cz.agents.agentdrive.highway.maneuver.HighwaySituation;
import cz.agents.agentdrive.highway.maneuver.LaneLeftManeuver;
import cz.agents.agentdrive.highway.maneuver.LaneRightManeuver;
import cz.agents.agentdrive.highway.maneuver.StraightManeuver;
import cz.agents.agentdrive.highway.storage.RoadObject;
import cz.agents.agentdrive.highway.storage.plan.Action;

public class GSDAgent extends RouteAgent {

    protected static final Logger logger = Logger.getLogger(GSDAgent.class);

    private final static double SAFETY_RESERVE = Configurator.getParamDouble("highway.safeDistanceAgent.safetyReserveDistance", 4.0);
    private final static double MAX_SPEED               = Configurator.getParamDouble("highway.safeDistanceAgent.maneuvers.maximalSpeed", 70.0  );
    private final static double MAX_SPEED_VARIANCE      = Configurator.getParamDouble("highway.safeDistanceAgent.maneuvers.maxSpeedVariance", 0.8 );

    private final static double LANE_SPEED_RATIO        = Configurator.getParamDouble("highway.safeDistanceAgent.laneSpeedRatio",            0.1);
    private final static long   PLANNING_TIME           = 1000;
    private final static int CIRCLE_AROUND_FOR_SEARCH = 5;
    private final static int ANGLE_TO_JUNCTION = 60;

    private final static int DISTANCE_TO_THE_JUNCTION = Configurator.getParamInt("highway.safeDistanceAgent.distanceToActivateNM", 400);
    private final static int CHECKING_DISTANCE = 500;
    protected int num_of_lines;

    private int numberOfCollisions=0;
    private CarManeuver currentManeuver = null;
    private boolean junctionMode = false;

    // maximal speed after variance application
    private final double initialMaximalSpeed = (RandomProvider.getRandom().nextDouble() - 0.5) * 2 *MAX_SPEED_VARIANCE * MAX_SPEED  + MAX_SPEED;
    private double maximalSpeed = initialMaximalSpeed;
    private ActualLanePosition myActualLanePosition;

    RoadNetwork roadNetwork;

    public GSDAgent(int id, RoadNetwork roadNetwork) {
        super(id);
        this.roadNetwork = roadNetwork;
        num_of_lines = navigator.getLane().getParentEdge().getLanes().keySet().size();
    }

    @Override
    public List<Action> agentReact() {
        return super.agentReact(plan());
    }

    public CarManeuver plan() {
        CarManeuver maneuver = null;
        RoadObject currState = sensor.senseCurrentState();
        // Simulator did not send update yet
        if (currState == null) {
            return null;
        }
        if(navigator.getLane() == null)
        {
            return null;
        }
        HighwaySituation situationPrediction = (HighwaySituation) getStatespace(currState);
        int lane = currState.getLaneIndex();
        double velocity = currState.getVelocity().length();
        long updateTime = (long) (currState.getUpdateTime() * 1000);
        // I am point zero so distance to me is 0
        CarManeuver acc   = new AccelerationManeuver  (lane, velocity, 0, updateTime);
        CarManeuver str   = new StraightManeuver      (lane, velocity, 0, updateTime);
        CarManeuver left  = new LaneLeftManeuver      (lane, velocity, 0, updateTime);
        CarManeuver right = new LaneRightManeuver     (lane, velocity, 0, updateTime);
        CarManeuver dec   = new DeaccelerationManeuver(lane, velocity, 0, updateTime);
        /*
        Order of maneuvers
        1. try to switch right
        2. try to accelerate
        3. try to not to change anything
        4. try to to switch left
        5. try to decelerate
        */
        if (!junctionMode && isSafeMan(currState, right, situationPrediction)) {
            maneuver = right;
        } else if (isSafeMan(currState, acc, situationPrediction)) {
            maneuver = acc;
        } else if (isSafeMan(currState, str, situationPrediction)) {
            maneuver = str;
        } else if (!junctionMode && isSafeMan(currState, left, situationPrediction)) {
            maneuver = left;
        } else if (isSafeMan(currState, dec, situationPrediction)) {
            maneuver = dec;
        } else {
            logger.info("Nothing is safe, shouldnt happen!");
            maneuver = dec;
        }
        currentManeuver = maneuver;
        logger.debug(maneuver);

        /*if(currState.getId() == 2 && currState.getUpdateTime() > 20000) {
            logger.info("Vehicle 2 now stops");
            maneuver = dec;
        }*/

        return maneuver;
    }
    private boolean isSafeMan(RoadObject state, CarManeuver man, HighwaySituation situation) {
        if (isHighwayCollision(state, man)) {
            return false;
        }
        if (isRulesCollision(man)) {
            return false;
        }
        HighwaySituation emptySituation = new HighwaySituation();
        emptySituation.clear();
        if (!isSafe(man, emptySituation))
            return false;

        if (man.getClass().equals(AccelerationManeuver.class)
                || man.getClass().equals(StraightManeuver.class)) {

            return isInSafeDistance(situation.getCarAheadMan(), man);
        } else if (man.getClass().equals(LaneLeftManeuver.class)) {
            return isInSafeDistance(situation.getCarLeftAheadMan(), man)
                    && isInSafeDistance(man, situation.getCarLeftMan());
        } else if (man.getClass().equals(LaneRightManeuver.class)) {
            return isInSafeDistance(situation.getCarRightAheadMan(), man)
                    && isInSafeDistance(man, situation.getCarRightMan());
        } else {
            return true;
        }
    }

    /**
     * Controls if there is a collision with the road structure.
     */
    private boolean isHighwayCollision(RoadObject state, CarManeuver man) {
        if (!canPerformManeuver(man)) {
            return true;
        }  else
            return false;
    }
    private double safeDistance(CarManeuver manAhead, CarManeuver manBehind, double safetyReserve) {
        // TODO get the a0 from the car
        double a0 = -1;
        double v0 = manBehind.getVelocityOut();
        double v1 = manAhead.getVelocityIn();
        double safeDist = safeDistance(a0, v0, v1, safetyReserve);
        return safeDist;
    }

    /**
     * calculation of the safe distance.
     */
    private double safeDistance(double a0, double v0, double v1) {
        double safeDist = (v1 * v1 - v0 * v0) / (2 * a0);
        return safeDist;
    }

    private double safeDistance(double a0, double v0, double v1, double safetyReserve) {
        double safeDist = safeDistance(a0, v0, v1) + safetyReserve;
        return safeDist;
    }

    private boolean isInSafeDistance(CarManeuver manAhead, CarManeuver manBehind) {
        if (manAhead == null || manBehind == null) return true;
        double realDist = manAhead.getPositionIn() - manBehind.getPositionOut();
        if(realDist < 0) return false; // maneuvers planned position out is before me.
        if (safeDistance(manAhead, manBehind, SAFETY_RESERVE) < realDist) return true;
        else return false;
    }

    public ArrayList<CarManeuver> getStatespace(RoadObject state) {
        Collection<RoadObject> cars = sensor.senseCars();
        long planningStartTime = sensor.getEventProcessor().getCurrentTime();
        ArrayList<CarManeuver> stateSpace = generateSS(state, cars, planningStartTime, planningStartTime + PLANNING_TIME);
        return stateSpace;
    }

    public boolean isCollision(CarManeuver carManeuver, ArrayList<CarManeuver> stateSpace) {

        for (int i = 0; i < stateSpace.size(); i++) {
            if (isCollision(carManeuver, stateSpace.get(i))) {
                return true;
            }
        }
        return false;
    }

    public boolean isCollision(CarManeuver carManeuver, CarManeuver carManeuver2) {

        if (isTimeIntersect(carManeuver, carManeuver2)
                && isPositionIntersect(carManeuver, carManeuver2)) {
            return true;
        } else
            return false;
    }

    public boolean isPositionIntersect(CarManeuver a, CarManeuver b) {

        // TODO compare maneuvers

        if (a.getPositionIn() > b.getPositionOut() || b.getPositionIn() > a.getPositionOut()) {
            return false;
        }
        if (a.getLaneIn() == b.getLaneIn() || a.getLaneIn() == b.getLaneOut()
                || a.getLaneOut() == b.getLaneIn() || a.getLaneOut() == b.getLaneOut()) {
            return true;
        }

        return false;

    }

    public boolean isTimeIntersect(CarManeuver carManeuver, CarManeuver carManeuver2) {
        long ret = getTimeConflict(carManeuver, carManeuver2);
        if (ret == -1)
            return false;
        else {
            return true;
        }
    }

    /*
     * Cooperative method returns conflictTime, if a conflict is found else return -1
     */
    public long getTimeConflict(CarManeuver carManeuver, CarManeuver carManeuver2) {

        long car1Start = carManeuver.getStartTime();
        long car1End = carManeuver.getEndTime();
        long car2Start = carManeuver2.getStartTime();
        long car2End = carManeuver2.getEndTime();

        if (carManeuver2.isInfiniteInTime())
            car2End = Long.MAX_VALUE;

        if ((car1Start > car2End) || (car2Start > car1End))
            return -1;
        else {
            if (car1Start > car2Start)
                return car1Start;
            else
                return car2Start;
        }
    }

    private boolean isSafe(CarManeuver carManeuver, ArrayList<CarManeuver> stateSpace) {
        CarManeuver man = new DeaccelerationManeuver(carManeuver);
        boolean ret = true;
        while (man.getVelocityOut() != 0.0) {
            if (isCollision(man, stateSpace)) {
                ret = false;
                break;
            }
            man = new DeaccelerationManeuver(man);
        }

        return ret;

    }

    private boolean laneChange(CarManeuver carManeuver){
        return carManeuver.getClass().equals(LaneLeftManeuver.class) || carManeuver.getClass().equals(LaneRightManeuver.class);
    }

    private boolean isRulesCollision(CarManeuver carManeuver) {
        if (laneChange(carManeuver)) {
            double minSpeedToTurn = 2;
            if (carManeuver.getVelocityIn() < minSpeedToTurn) {
                return true;
            }
        }
        // TODO check - observed that StraightManeuver accelerates vehicle
        double laneMaxSpeed = maximalSpeed + carManeuver.getLaneIn() * LANE_SPEED_RATIO * maximalSpeed;
        return carManeuver.getVelocityOut() > laneMaxSpeed;
    }

    public boolean canPerformManeuver(CarManeuver carManeuver) {
        // always allow a finishing maneuver
        if (carManeuver.isFinishing()) {
            return true;
        }

        int laneOut = carManeuver.getExpectedLaneOut();
        if (laneOut < 0) {
            return false;
        }
        if (laneOut >= navigator.getLane().getParentEdge().getLanes().keySet().size()) {
            return false;
        }

        return true;
    }


    /**
     * This method gets vehicles around and generates the HighwaySituation that is used for reasoning.
     * @param state This is the RoadObject of the operating vehicles.
     * @param cars Vehicles nearby.
     * @param from Starting time
     * @param to Ending time.
     * @return HighwaySituation
     */
    public HighwaySituation generateSS(RoadObject state, Collection<RoadObject> cars, long from, long to) {

        HighwaySituation situationPrediction = new HighwaySituation();

        logger.debug("GenerateSS for "+state.getId());
        Collection<RoadObject> nearCars = new Vector<RoadObject>();
        myActualLanePosition = roadNetwork.getActualPosition(state.getPosition());
        navigator.setActualPosition(myActualLanePosition);
        Lane myLane = myActualLanePosition.getLane();
        Edge myEdge = myActualLanePosition.getEdge();

        logger.debug("myLane = "+myLane.getIndex());
        num_of_lines = myEdge.getLanes().size();
        int myIndexOnRoute= myActualLanePosition.getIndex();//   getNearestWaipointIndex(state,myLane);

        //removing too far cars and myself from the collection
        for (RoadObject entry : cars) {
            float distanceToSecondCar = entry.getPosition().distance(state.getPosition());
            if(distanceToSecondCar > CHECKING_DISTANCE || state.getPosition().equals(entry.getPosition()))
            {
                continue;
            }
            else
            {
                if(distanceToSecondCar < 2.24)
                    numberOfCollisions++;
                nearCars.add(entry);
            }
        }
        // Main logic, first there is a check if there is a junction nearby. If so the junction mode is enabled. If not,
        //vehicle is driven by standart Safe-distance agent
        Lane entryLane;
        ActualLanePosition entryActualLanePosition;
        Junction myNearestJunction = roadNetwork.getJunctions().get(myEdge.getTo());
        Point2f junctionwaypoint = myNearestJunction.getCenter();
        boolean nearTheJunction = (convertPoint3ftoPoint2f(state.getPosition()).distance(junctionwaypoint) < DISTANCE_TO_THE_JUNCTION && myNearestJunction.getIncLanes().size() > 1);
        //distance from the junction, should be determined by max allowed speed on the lane.
        for(RoadObject entry : nearCars) {
            Point2f intersectionWaypoint = junctionwaypoint;
            ArrayList<CarManeuver> predictedManeuvers;
            entryActualLanePosition =roadNetwork.getActualPosition(entry.getPosition());
            entryLane = entryActualLanePosition.getLane();
            if(myNearestJunction.equals(roadNetwork.getJunctions().get(entryLane.getParentEdge().getTo()))) {
            // if operating vehicle and other vehicle is heading to the same junction
                if (nearTheJunction)
                {
                    junctionMode = true;

                    //This part of code requires the knowledge of the long-term plan of the other vehicle.
                    // It determines if the vehicles croses their paths at the junction.
                    intersectionWaypoint = actualiseIntersectionWaypoint(entry,myLane,entryLane,junctionwaypoint);
                    if(intersectionWaypoint == null) continue;
                    // Transformation from the 2D space into the 1D space. Vehicles are virtually put on the one line
                    // to the junction by their distance to the junction.
                    float myDistance = getDistanceToTheJunction(myIndexOnRoute,myLane,intersectionWaypoint);
                    float entryDistance = getDistanceToTheJunction(entryActualLanePosition.getIndex(),entryLane,intersectionWaypoint);
                    if (entryDistance < myDistance) {
                        StraightManeuver man = new StraightManeuver(entry.getId(), entry.getVelocity().length(), myDistance - entryDistance, (long) (entry.getUpdateTime() * 1000));
                        situationPrediction.trySetCarAheadManeuver(man);
                    }
                    else if(entryDistance == myDistance) //solution when distances are the same.
                    {
                        Random rand = new Random();
                        if(rand.nextFloat() > 0.5)
                        {
                            StraightManeuver man = new StraightManeuver(entry.getId(), entry.getVelocity().length(), myDistance - entryDistance, (long) (entry.getUpdateTime() * 1000));
                            situationPrediction.trySetCarAheadManeuver(man);
                        }
                    }
                }
                else {
                    // other vehicle is heading to the same junction but the junction is not close.
                    //this is the classical Safe-distance method, all five states can be set.
                    junctionMode = false;
                    if (entryLane.getParentEdge().equals(myEdge)) {

                        predictedManeuvers = getPlannedManeuvers(state, myActualLanePosition, entry, entryActualLanePosition, from, to, null);
                        situationPrediction.addAll(predictedManeuvers);

                        CarManeuver man = predictedManeuvers.get(0);

                        int entryNearestWaipoint = entryActualLanePosition.getIndex(); // finding nearest waipoint on the road.
                        if (myLane.getLaneId().equals(entryLane.getLaneId())) {
                            if (entryNearestWaipoint > myIndexOnRoute) {
                                situationPrediction.trySetCarAheadManeuver(man);
                            }
                        } else {
                            if (entryNearestWaipoint < myIndexOnRoute) {
                                if (myLane.getIndex() - entryLane.getIndex() == -1) {
                                    situationPrediction.trySetCarLeftMan(man);
                                } else if (myLane.getIndex() - entryLane.getIndex() == 1) {
                                    situationPrediction.trySetCarRightMan(man);
                                }
                            } else {
                                if (myLane.getIndex() - entryLane.getIndex() == -1) {
                                    situationPrediction.trySetCarLeftAheadMan(man);
                                } else if (myLane.getIndex() - entryLane.getIndex() == 1) {
                                    situationPrediction.trySetCarRightAheadMan(man);
                                }
                            }
                        }

                    }
                }
            }
            if (!entryLane.getParentEdge().equals(myEdge)) { // This is for checking vehicles behind the junction.
                List<Edge> remE = navigator.getFollowingEdgesInPlan();
                for (Edge planned : remE) {
                    if (planned.equals(entryLane.getParentEdge())) {
                        predictedManeuvers = getPlannedManeuvers(state, myActualLanePosition, entry, entryActualLanePosition, from, to, remE);
                        situationPrediction.addAll(predictedManeuvers);
                        CarManeuver man = predictedManeuvers.get(0);
                        //TODO Improve this part to allow 2 lanes throw junction
                        if((Math.abs(state.getLaneIndex() - entry.getLaneIndex()) <= 1))
                        {
                            situationPrediction.trySetCarAheadManeuver(man);
                            situationPrediction.trySetCarLeftAheadMan(man);
                            situationPrediction.trySetCarRightAheadMan(man);
                        }
                    }
                }
                continue;
            }
        }
        return situationPrediction;
    }

    /**
     * Method for fining nearest waipoint on the Lane
     * @param state Vehicle's state
     * @param myLane Vehicle's lane.
     * @return index on the lane.
     */
    @Deprecated
    private int getNearestWaipointIndex(RoadObject state,Lane myLane)
    {
        int myIndexOnRoute = 0;
        while(distanceP2P2(myLane.getInnerPoints().get(myIndexOnRoute), new Point2f(state.getPosition().x, state.getPosition().y))  > CIRCLE_AROUND_FOR_SEARCH) // Magical value
        {
            myIndexOnRoute++;
            if(myLane.getInnerPoints().size() == myIndexOnRoute)
            {
                myIndexOnRoute--;
                break;
            }
        }
        while(distanceP2P2(myLane.getInnerPoints().get(myIndexOnRoute), new Point2f(state.getPosition().x, state.getPosition().y))  <= CIRCLE_AROUND_FOR_SEARCH)
        {
            myIndexOnRoute++;
            if(myLane.getInnerPoints().size() == myIndexOnRoute)
            {
                myIndexOnRoute--;
                break;
            }
        }
        if(myIndexOnRoute >= myLane.getInnerPoints().size())
            myIndexOnRoute=myLane.getInnerPoints().size()-1;
        // logger.debug("nearest waipoint id " + myIndexOnRoute);
        return  myIndexOnRoute;
    }

    /**
     * Calculate distance between two points
     * @param innerPoint
     * @param position
     * @return distance
     */
    public float distanceP2P2(Point2f innerPoint, Point2f position) {
        return innerPoint.distance(position);
    }

    public ArrayList<CarManeuver> getPlannedManeuvers(RoadObject me,ActualLanePosition myActualLanePosition,RoadObject car,ActualLanePosition otherActualLanePosition, long from, long to,List<Edge> rem) {

        ArrayList<CarManeuver> plan = new ArrayList<CarManeuver>();
        // TODO add a part of plan that is between from and to
        CarManeuver lastManeuver;
        lastManeuver = new StraightManeuver(car.getLaneIndex(), car.getVelocity().length(),
                getDistanceBetweenTwoRoadObjects(me,myActualLanePosition,car,otherActualLanePosition,rem), (long) (car.getUpdateTime() * 1000));
        plan.add(lastManeuver);
        while (lastManeuver.getEndTime() <= to) {
            lastManeuver = new StraightManeuver(lastManeuver);
            plan.add(lastManeuver);
        }

        return plan;
    }
    /**
     * Distance betwenn two road objects is calculated by finding two nearest waipoints of the cars and distance between them,
     * on the "same" line.
     * @param me my road state
     * @param myActualLanePosition
     * @param other other's vehicle road state.
     * @param otherActualLanePosition
     * @return distance
     */
    private double getDistanceBetweenTwoRoadObjects(RoadObject me,ActualLanePosition myActualLanePosition,RoadObject other,ActualLanePosition otherActualLanePosition,List<Edge> rem) {
       // int nearestA = getNearestWaipointCloseEnough(me, myLane);
       // int nearestB = getNearestWaipointCloseEnough(other, otherLane);
        Lane myLane = myActualLanePosition.getLane();
        Lane otherLane = otherActualLanePosition.getLane();
        int nearestA = myActualLanePosition.getIndex();
        int nearestB = otherActualLanePosition.getIndex();

        Edge myEdg = myLane.getParentEdge();
        Edge his = otherLane.getParentEdge();
        double distance = 0;
        int maxSize = 0;
        if (myEdg.equals(his)) {
            if (myLane.getInnerPoints().size() < otherLane.getInnerPoints().size())  //two lines in the same edge with diferent number of waipoints, typicaly in curves
            {
                maxSize = myLane.getInnerPoints().size();
            } else {
                maxSize = otherLane.getInnerPoints().size();
            }
            if (nearestA < nearestB)// car is ahead
            {
                for (int i = nearestA + 1; i <= nearestB && i < maxSize; i++) {
                    distance += myLane.getInnerPoints().get(i - 1).distance(myLane.getInnerPoints().get(i));
                }
                return distance;
            } else if (nearestA > nearestB) {
                for (int i = nearestB + 1; i <= nearestA && i < maxSize; i++) {
                    distance += myLane.getInnerPoints().get(i - 1).distance(myLane.getInnerPoints().get(i));
                }
                return -distance;
            } else
                return 0;
        }
        else
        {
            // calculate distance to the vehicle on the edge that is on my plan.
            int distC = 0;
            for (int i = nearestA + 1; i < myLane.getInnerPoints().size();i++) {
                distC += myLane.getInnerPoints().get(i - 1).distance(myLane.getInnerPoints().get(i));
            }
            for(int i =0;i<rem.size();i++)
            {
                if(rem.get(i).equals(his))
                {
                    for (int d = 1; d <= nearestB;d++) {
                        distC += otherLane.getInnerPoints().get(d - 1).distance(otherLane.getInnerPoints().get(d));
                    }
                    break;
                }
                //TODO FIX MORE LANES LENGHT
                LaneImpl tem = rem.get(i).getLaneByIndex(0);
                for (int d = 1; d < tem.getInnerPoints().size();d++) {
                    distC += tem.getInnerPoints().get(d - 1).distance(tem.getInnerPoints().get(d));
                }

            }
            return distC;
        }
    }

    /**
     * Method for finding nearest waypoint
     * @param me Road object of operating vehicle
     * @param myLane vehicle's lane
     * @return index on the lane.
     */
    public int getNearestWaypointCloseEnough(RoadObject me, Lane myLane)
    {
        Point2f myPosition = convertPoint3ftoPoint2f(me.getPosition());
        Point2f innerPoint = myLane.getInnerPoints().get(0);
        int i=0;
        while((!pointCloseEnough(innerPoint, myPosition, convertVector3ftoVector2f(me.getVelocity())) && i<myLane.getInnerPoints().size()))
        {

            innerPoint = myLane.getInnerPoints().get(i);
            i++;

        }
        //TODO FIX THIS
        if(i >= myLane.getInnerPoints().size())
            i=myLane.getInnerPoints().size()-1;
        return i;
    }
    public Point2f convertPoint3ftoPoint2f(Point3f point)
    {
        return new Point2f(point.x,point.y);
    }
    public Vector2f convertVector3ftoVector2f(Vector3f vec)
    {
        return new Vector2f(vec.x,vec.y);
    }

    /**
     * This method is for measuring number of collisons by HighwayStorage.
     * @return number of vehicle's collisions
     */
    public int getNumberOfColisions()
    {
        return numberOfCollisions;
    }
    private float getDistanceToTheJunction(int nearest, Lane myLane,Point2f intersectionWaypoint) {
        float distance =0;
        int maxSize =myLane.getInnerPoints().size();
        for (int i = nearest + 1; i < maxSize; i++) {
            distance += myLane.getInnerPoints().get(i - 1).distance(myLane.getInnerPoints().get(i));
        }
        return distance + myLane.getInnerPoints().get(myLane.getInnerPoints().size()-1).distance(intersectionWaypoint);
    }
    private Point2f actualiseIntersectionWaypoint(RoadObject entry,Lane myLane,Lane entryLane,Point2f junctionWaypoint)
    {
        Point2f intersectionWaypoint = junctionWaypoint;
        Map<Integer, Agent> agents = sensor.getAgents();
        GSDAgent entryAgent = (GSDAgent)agents.get(entry.getId());
        //calculation of optimised intersection point, if not found, other vehicle is ignored.
        if(!entryAgent.navigator.getFollowingEdgesInPlan().isEmpty() && !navigator.getFollowingEdgesInPlan().isEmpty()) {
            Edge entryNextEdge = entryAgent.navigator.getFollowingEdgesInPlan().iterator().next();
            Edge myNextEdge = navigator.getFollowingEdgesInPlan().iterator().next();

            Point2f p0 = myLane.getInnerPoints().get(myLane.getInnerPoints().size() - 1);
            Point2f p1 = myNextEdge.getLaneByIndex(0).getInnerPoints().get(0);
            Point2f p2 = entryLane.getInnerPoints().get(entryLane.getInnerPoints().size() - 1);
            Point2f p3 = entryNextEdge.getLaneByIndex(0).getInnerPoints().get(0);

            Point2f newCenter = isColision(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
            if (newCenter == null && !myLane.equals(entryLane)) {
                intersectionWaypoint = null;
            } else if (newCenter != null && !myLane.equals(entryLane)) {
                if (junctionWaypoint.distance(newCenter) < 30) {
                    intersectionWaypoint = new Point2f((newCenter.x + junctionWaypoint.x) / 2, (newCenter.y + junctionWaypoint.y) / 2);
                }
            }
        }
        return intersectionWaypoint;
    }
    /**
     * This method is for detection of intersection between two lane segments
     * @param p0_x x coordinate of the starting point of the first lane segment
     * @param p0_y y coordinate of the starting point of the first lane segment
     * @param p1_x x coordinate of the ending point of the first lane segment
     * @param p1_y y coordinate of the ending point of the first lane segment
     * @param p2_x x coordinate of the starting point of the second lane segment
     * @param p2_y y coordinate of the starting point of the second lane segment
     * @param p3_x x coordinate of the ending point of the second lane segment
     * @param p3_y y coordinate of the ending point of the second lane segment
     * @return null if no intersection point found or the point of intersection.
     */
    private Point2f isColision(float p0_x, float p0_y, float p1_x, float p1_y,
                               float p2_x, float p2_y, float p3_x, float p3_y)
    {
        float s1_x, s1_y, s2_x, s2_y;
        s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
        s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

        float s, t;
        s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
        t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            float i_x = p0_x + (t * s1_x);
            float i_y = p0_y + (t * s1_y);
            return new Point2f(i_x,i_y);
        }
        return null; // No collision
    }

    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        this.sensor.registerReaction(new Reaction() {
            public void react(Event event) {
                if (event.getType().equals(HighwayEventType.UPDATED)) {
                    actuator.act(agentReact());
                }
            }
        });

    }
}
