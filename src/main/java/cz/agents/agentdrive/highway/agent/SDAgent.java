package cz.agents.agentdrive.highway.agent;

import cz.agents.agentdrive.highway.environment.RandomProvider;
import cz.agents.agentdrive.highway.environment.roadnet.*;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.maneuver.*;
import cz.agents.agentdrive.highway.storage.RoadObject;
import cz.agents.agentdrive.highway.storage.plan.Action;
import cz.agents.agentdrive.highway.storage.plan.ManeuverAction;
import cz.agents.agentdrive.highway.storage.plan.WPAction;
import cz.agents.alite.configurator.Configurator;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import javax.vecmath.Point2d;
import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Vector;

public class SDAgent extends RouteAgent {

    protected static final Logger logger = Logger.getLogger(SDAgent.class);

    private final static double DISTANCE_TO_ACTIVATE_NM = Configurator.getParamDouble("highway.safeDistanceAgent.distanceToActivateNM", 300.0);
    private final static double SAFETY_RESERVE = Configurator.getParamDouble("highway.safeDistanceAgent.safetyReserveDistance", 4.0);
    private final static double MAX_SPEED = Configurator.getParamDouble("highway.safeDistanceAgent.maneuvers.maximalSpeed", 70.0);
    private final static double MAX_SPEED_VARIANCE = Configurator.getParamDouble("highway.safeDistanceAgent.maneuvers.maxSpeedVariance", 0.8);

    protected final static double LANE_SPEED_RATIO = Configurator.getParamDouble("highway.safeDistanceAgent.laneSpeedRatio", 0.1);
    private final static long PLANNING_TIME = 1000;
    protected int num_of_lines;
    protected ActualLanePosition myActualLanePosition;
    protected CarManeuver currentManeuver = null;
    protected HighwaySituation currentHighwaySituation = null;
    protected int edgeIndex = 0;

    // maximal speed after variance application
    protected final double initialMaximalSpeed = (RandomProvider.getRandom().nextDouble() - 0.5) * 2 * MAX_SPEED_VARIANCE * MAX_SPEED + MAX_SPEED;
    protected double maximalSpeed = initialMaximalSpeed;

    public List<Action> agentReact() {
        return super.agentReact(plan());
    }


    private Action man2Action(CarManeuver man) {
        if (man == null) {

            return new WPAction(id, 0d, getInitialPosition(), 0);
        }
        return new ManeuverAction(sensor.getId(), man.getStartTime() / 1000.0, man.getVelocityOut(), man.getLaneOut(), man.getDuration());
    }

    protected RoadNetwork roadNetwork;

    public SDAgent(int id, RoadNetwork roadNetwork) {
        super(id);
        this.roadNetwork = roadNetwork;
        logger.setLevel(Level.DEBUG);
        System.out.println("init max speed " + initialMaximalSpeed);

        num_of_lines = navigator.getLane().getParentEdge().getLanes().keySet().size();
    }


    public void predictNext(HighwaySituation situationPrediction, RoadObject car, long predictionEndTime) {
        CarManeuver man = new StraightManeuver(car.getLaneIndex(), car.getVelocity().length(), getDistance(car), (long) (car.getUpdateTime() * 1000));
        if (!isCollision(man, situationPrediction)) {
            situationPrediction.add(man);
        }
    }

    public HighwaySituation getHighwaySituation() {
        return currentHighwaySituation;
    }

    public CarManeuver plan() {
        CarManeuver maneuver = null;

        RoadObject currState = sensor.senseCurrentState();
        if (currState == null) {
            logger.debug("No plan for car " + this.getName());
            return null;
        }
        if (navigator.getLane() == null) {
            logger.debug("No plan for car " + this.getName());
            return null;
        }
        int myLaneIndex = navigator.getLane().getIndex();
        for (LaneImpl l : navigator.getLane().getParentEdge().getLanes().values()) {
            if (l.getInnerPoints().contains(new Point2f(currState.getPosition().x, currState.getPosition().y))) {
                myLaneIndex = l.getIndex();
            }
        }
        while (myLaneIndex < navigator.getLane().getIndex()) {
            navigator.changeLaneRight();
        }
        while (myLaneIndex > navigator.getLane().getIndex()) {
            navigator.changeLaneLeft();
        }

        // Simulator did not send update yet
        if (currState == null) {
            return null;
        }

        logger.debug("Startnode: " + currState);
        HighwaySituation situationPrediction = (HighwaySituation) getStatespace(currState);
        logger.debug("SS: " + situationPrediction);
        logger.debug("Situation: " + situationPrediction);
        logger.debug("Navigator: " + navigator.getRoutePoint());

        int lane = currState.getLaneIndex();
        double velocity = currState.getVelocity().length();

        Point2f edgeBeginPoint = navigator.getLane().getInnerPoints().get(0);
        double distance = 0;//getDistance(currState, edgeBeginPoint); //transGeoToDistance(currState.getPosition());

        long updateTime = (long) (currState.getUpdateTime() * 1000);

        CarManeuver acc = new AccelerationManeuver(lane, velocity, distance, updateTime);
        CarManeuver str = new StraightManeuver(lane, velocity, distance, updateTime);
        CarManeuver left = new LaneLeftManeuver(lane, velocity, distance, updateTime);
        CarManeuver right = new LaneRightManeuver(lane, velocity, distance, updateTime);
        CarManeuver dec = new DeaccelerationManeuver(lane, velocity, distance, updateTime);

        int preferredLane = getPreferredLane(currState);

        if (isNarrowingMode(currState)) {
            logger.debug("PreferredLane: " + preferredLane);
            logger.debug("Narrowing mode activated");
            CarManeuver preferredMan = null;
            if (preferredLane < currState.getLaneIndex()) {
                preferredMan = right;
            } else if (preferredLane > currState.getLaneIndex()) {
                preferredMan = left;
            }
            if (preferredMan != null && isSafeMan(currState, preferredMan, situationPrediction)) {
                maneuver = preferredMan;
            } else if (isSafeMan(currState, acc, situationPrediction)) {
                maneuver = acc;
            } else if (isSafeMan(currState, str, situationPrediction)) {
                maneuver = str;
            } else if (isSafeMan(currState, dec, situationPrediction)) {
                maneuver = dec;
            } else {
                maneuver = dec;
            }
        } else { // Not narrowingMode
            //performing changing lane?
            if (currentManeuver != null && currentManeuver.getClass().equals(LaneLeftManeuver.class) && isSafeMan(currState, left, situationPrediction)) {
                maneuver = left;
            } else if (currentManeuver != null && currentManeuver.getClass().equals(LaneRightManeuver.class) && isSafeMan(currState, right, situationPrediction)) {
                maneuver = right;
            } else {

                if (isSafeMan(currState, right, situationPrediction)) {
                    maneuver = right;
                } else if (isSafeMan(currState, acc, situationPrediction)) {
                    maneuver = acc;
                } else if (isSafeMan(currState, str, situationPrediction)) {
                    maneuver = str;
                } else if (isSafeMan(currState, left, situationPrediction)) {
                    maneuver = left;
                } else if (isSafeMan(currState, dec, situationPrediction)) {
                    maneuver = dec;
                } else {
                    logger.info("Nothing is safe, shouldn't happen!");
                    maneuver = dec;
                }
            }
        }
        logger.info("Planned maneuver for carID " + currState.getId() + " " + maneuver);
        currentManeuver = maneuver;
        currentHighwaySituation = situationPrediction;
        return maneuver;
    }

    @Deprecated
    private double transGeoToDistance(double x, double y) {
        return sensor.getRoadDescription().distance(new Point2d(x, y));
    }

    @Deprecated
    protected double transGeoToDistance(Point3f position) {
        return transGeoToDistance(position.x, position.y);
    }

    private int getPreferredLane(RoadObject startNode) {
        // double dist = getDistance(startNode);
        // if(dist>100 && dist <=300)return 0;
        // else if(dist>300)return 1;
        // else return 0;

        double distance = 1000;
        while (distance >= 200) {
            for (int lane = 1; lane <= 2; lane++) {
                if (isLaneGoingOn(getDistance(startNode), distance, lane)) {
                    return lane;
                }

                distance -= 200;
            }
        }
        return 1;
    }

    private boolean isNarrowingMode(RoadObject state) {
        if (Configurator.getParamBool("highway.safeDistanceAgent.narrowingModeActive", false).equals(false))
            return false;

        int lane = state.getLaneIndex();
        double distance = getDistance(state);

        // following is universal
        boolean myLaneEnding = !isLaneGoingOn(distance, DISTANCE_TO_ACTIVATE_NM, lane);
        int lastLane = state.getLaneIndex() - 1;
        boolean leftLaneEnding = lane != lastLane
                && !isLaneGoingOn(distance, DISTANCE_TO_ACTIVATE_NM, lane + 1);
        boolean rightLaneEnding = lane != 0
                && !isLaneGoingOn(distance, DISTANCE_TO_ACTIVATE_NM, lane - 1);
        if (myLaneEnding)
            logger.debug("myLaneEnding");
        if (leftLaneEnding)
            logger.debug("leftLaneEnding");
        if (rightLaneEnding)
            logger.debug("rightLaneEnding");
        return myLaneEnding || leftLaneEnding || rightLaneEnding;

    }

    private boolean isLaneGoingOn(double position, double distance, int lane) {
        Collection<RoadObject> obstacles = sensor.senseObstacles();
        for (RoadObject obs : obstacles) {
            if (obs.getLaneIndex() != lane)
                continue;
            double obsDist = getDistance(obs);
            if (obsDist > position && obsDist < position + distance) {
                return false;
            }
        }
        return true;
    }

    @Deprecated
    protected double getDistance(RoadObject state) {
        return transGeoToDistance(state.getPosition());
    }

    protected boolean isSafeMan(RoadObject state, CarManeuver man, HighwaySituation situation) {
        boolean narrowingMode = isNarrowingMode(state);

        if (isHighwayCollision(state, man)) {
            logger.info(state + " Highway Collision detected!" + man);
            return false;
        }
        if (isRulesCollision(man)) {
            logger.info("Rules Collision detected! " + man);
            return false;

        }
        HighwaySituation emptySituation = new HighwaySituation();
        emptySituation.clear();
        if (!isSafe(man, emptySituation)) {
            logger.info("Empty situation is not safe");
            return false;
        }
        if (man.getClass().equals(AccelerationManeuver.class)
                || man.getClass().equals(StraightManeuver.class)) {

            return isInSafeDistance(situation.getCarAheadMan(), man)// sufficient if not narrowing
                    && (!narrowingMode || (isInSafeDistance(situation.getCarLeftAheadMan(), man) || (situation
                    .getCarLeftAheadMan() != null && situation.getCarLeftAheadMan()
                    .getVelocityOut() == 0.0)) // if narrowingMode check also cars in left
                    // lane
                    && (!narrowingMode || (isInSafeDistance(
                    situation.getCarRightAheadMan(), man) || (situation
                    .getCarRightAheadMan() != null && situation
                    .getCarRightAheadMan().getVelocityOut() == 0.0))));// if
            // narrowingMode
            // check also
            // cars in
            // right lane

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

    private boolean isHighwayCollision(RoadObject state, CarManeuver man) {
        if (!canPerformManeuver(man)) {
            return true;
        } else if (!isLaneGoingOn(man.getPositionOut(), safeDistance(man, SAFETY_RESERVE), state.getLaneIndex())) {
            return true;
        } else
            return false;
    }

    private double safeDistance(CarManeuver man, double safetyReserve) {
        // TODO get the a0 from the car
        double a0 = -1;
        double v0 = man.getVelocityOut();
        double v1 = 0;
        return safeDistance(a0, v0, v1, safetyReserve);
    }

    private double safeDistance(CarManeuver manAhead, CarManeuver manBehind, double safetyReserve) {
        // TODO get the a0 from the car
        double a0 = -1;
        double v0 = manBehind.getVelocityOut();
        double v1 = manAhead.getVelocityIn();
        double safeDist = safeDistance(a0, v0, v1, safetyReserve);
        return safeDist;
    }

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
        if (realDist < 0) return false; // maneuvers planned position out is before me.
        if (safeDistance(manAhead, manBehind, SAFETY_RESERVE) < realDist) return true;
        else return false;
    }

    public ArrayList<CarManeuver> getStatespace(RoadObject state) {
        Collection<RoadObject> cars = sensor.senseCars();
        long planningStartTime = sensor.getCurrentTime();
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


    private boolean laneChange(CarManeuver carManeuver) {
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
        //if(carManeuver.getClass().equals(AccelerationManeuver.class) || carManeuver.getClass().equals(StraManeuver.class)){
        double laneMaxSpeed = maximalSpeed + carManeuver.getLaneIn() * LANE_SPEED_RATIO * maximalSpeed;
        logger.info("laneMasSpeed= " + laneMaxSpeed + " maximalSpeed=" + maximalSpeed + " speedVariance=" + MAX_SPEED_VARIANCE + " MAX_SPEED conf = " + MAX_SPEED);
        return carManeuver.getVelocityOut() > laneMaxSpeed;

//    	boolean ret = false;

        // what does this mean??
//        else if (carManeuver.getClass().equals(AccelerationManeuver.class)
//                && carManeuver.getVelocityIn() < 10.0) {
//            if (isCollision(new AccelerationManeuver(carManeuver), getStatespace())) {
//                ret = true;
//            }
//        }
//        return ret;
    }

    public boolean canPerformManeuver(CarManeuver carManeuver) {
        // always allow a finishing maneuver
        if (carManeuver.isFinishing()) {
            return true;
        }

        int laneOut = carManeuver.getExpectedLaneOut();
        if (laneOut < 0) {
            logger.debug("LaneOut= " + laneOut);
            return false;
        }
        if (laneOut >= navigator.getLane().getParentEdge().getLanes().keySet().size()) {
            logger.debug("laneOut = " + laneOut);
            return false;
        }

        return true;
    }


    public HighwaySituation generateSS(RoadObject state, Collection<RoadObject> cars, long from, long to) {

        HighwaySituation situationPrediction = new HighwaySituation();
        for (int i = 0; i < navigator.getRoute().size(); i++) {
            if (navigator.getRoute().get(i) == navigator.getLane().getParentEdge()) {
                edgeIndex = i;
                logger.debug("Index of an edge in a route is " + edgeIndex);
                break;
            }
        }
        int lastIndex = myActualLanePosition == null ? 0 : myActualLanePosition.getIndex();
        logger.debug("GenerateSS for " + state.getId());
        Collection<RoadObject> nearCars = new Vector<RoadObject>();
        myActualLanePosition = roadNetwork.getActualPosition(state.getPosition());
        navigator.setActualPosition(myActualLanePosition);

        checkCorrectRoute();

        //System.out.println(state.getPosition() + " " + navigator.getLane().getLaneId());
        //System.out.println(navigator.getRoute().contains());
        //  System.out.println(navigator.getLane().getInnerPoints());
        Lane myLane = myActualLanePosition.getLane();
        Edge myEdge = myActualLanePosition.getEdge();

        logger.debug("myLane = " + myLane.getIndex());
        num_of_lines = myEdge.getLanes().size();
        int myIndexOnRoute = myActualLanePosition.getIndex();//   getNearestWaipointIndex(state,myLane);

        //removing too far cars and myself from the collection
//        for (RoadObject entry : cars) {
//            float distanceToSecondCar = entry.getPosition().distance(state.getPosition());
//            if (distanceToSecondCar > CHECKING_DISTANCE || state.getPosition().equals(entry.getPosition())) {
//                continue;
//            } else {
//                if (distanceToSecondCar < 2.24)
//                    numberOfCollisions++;
//                nearCars.add(entry);
//            }
//        }
        // Main logic, first there is a check if there is a junction nearby. If so the junction mode is enabled. If not,
        //vehicle is driven by standart Safe-distance agent
        Lane entryLane;
        ActualLanePosition entryActualLanePosition;
        Junction myNearestJunction = roadNetwork.getJunctions().get(myEdge.getTo());
        Point2f junctionwaypoint = myNearestJunction.getCenter();
        //boolean nearTheJunction = (convertPoint3ftoPoint2f(state.getPosition()).distance(junctionwaypoint) < DISTANCE_TO_THE_JUNCTION && myNearestJunction.getIncLanes().size() > 1);
        //distance from the junction, should be determined by max allowed speed on the lane.
        for (RoadObject entry : cars) {
            Point2f intersectionWaypoint = junctionwaypoint;
            ArrayList<CarManeuver> predictedManeuvers;
            entryActualLanePosition = roadNetwork.getActualPosition(entry.getPosition());
            entryLane = entryActualLanePosition.getLane();
            if (myNearestJunction.equals(roadNetwork.getJunctions().get(entryLane.getParentEdge().getTo()))) {
                // if operating vehicle and other vehicle is heading to the same junction
                // other vehicle is heading to the same junction but the junction is not close.
                //this is the classical Safe-distance method, all five states can be set.
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
            if (!entryLane.getParentEdge().equals(myEdge)) { // This is for checking vehicles behind the junction.
                List<Edge> remE = navigator.getFollowingEdgesInPlan();
                for (Edge planned : remE) {
                    if (planned.equals(entryLane.getParentEdge())) {
                        predictedManeuvers = getPlannedManeuvers(state, myActualLanePosition, entry, entryActualLanePosition, from, to, remE);
                        situationPrediction.addAll(predictedManeuvers);
                        CarManeuver man = predictedManeuvers.get(0);
                        //TODO Improve this part to allow 2 lanes throw junction
                        if ((Math.abs(state.getLaneIndex() - entry.getLaneIndex()) <= 1)) {
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

    private double getDistance(RoadObject state, Point2f edgeBeginPoint) {
        return sensor.getRoadDescription().distance(new Point2d(state.getPosition().x, state.getPosition().y), edgeBeginPoint);
    }


    protected boolean checkCorrectRoute() {
        if (!navigator.getRoute().contains(myActualLanePosition.getEdge())) {
            logger.warn("Agent is on a route it should not be!  ");
            /*
            This can happen when a car is crossing a junction and method getActualPosition return position
            on a lane it's just crossing, but this lane is not connected to actual edge car is trying to get.
             */
            if (edgeIndex + 1 < navigator.getRoute().size()) {
                myActualLanePosition = new ActualLanePosition(navigator.getRoute().get(edgeIndex + 1).getLaneByIndex(0), 0);

            } else {
                myActualLanePosition = new ActualLanePosition(navigator.getRoute().get(edgeIndex).getLaneByIndex(0), 0);
            }
            navigator.setActualPosition(myActualLanePosition);
            return false;
        }
        return true;
    }

    public ArrayList<CarManeuver> getPlannedManeuvers(RoadObject me, ActualLanePosition myActualLanePosition, RoadObject car, ActualLanePosition otherActualLanePosition, long from, long to, List<Edge> rem) {

        ArrayList<CarManeuver> plan = new ArrayList<CarManeuver>();
        // TODO add a part of plan that is between from and to
        CarManeuver lastManeuver;
        lastManeuver = new StraightManeuver(car.getLaneIndex(), car.getVelocity().length(),
                Utils.getDistanceBetweenTwoRoadObjects(me, myActualLanePosition, car, otherActualLanePosition, rem), (long) (car.getUpdateTime() * 1000));
        plan.add(lastManeuver);
        while (lastManeuver.getEndTime() <= to) {
            lastManeuver = new StraightManeuver(lastManeuver);
            plan.add(lastManeuver);
        }

        return plan;
    }

    public ArrayList<CarManeuver> getPlannedManeuvers(RoadObject car, long from, long to) {
        // if has no plan -> predict TODO use more sophisticated prediction - not
        // here, on all the statespace
        ArrayList<CarManeuver> plan = new ArrayList<CarManeuver>();
        // TODO add a part of plan that is between from and to
        CarManeuver lastManeuver;
//      there is no sensing of braking and directional lights
//        boolean braking = false;
//        String turning = null;
//        if (braking) {
//            lastManeuver = new DeaccelerationManeuver(car.getLaneIndex(), car.getVelocity().length(), getDistance(car),
//                    (long) (car.getUpdateTime() * 1000));
//
//        } else if (turning != null) {
//            if (turning.equals("left")) {
//                lastManeuver = new LaneLeftManeuver(car.getLaneIndex(), car.getVelocity().length(), getDistance(car),
//                        (long) (car.getUpdateTime() * 1000));
//            } else {
//                lastManeuver = new LaneRightManeuver(car.getLaneIndex(), car.getVelocity().length(), getDistance(car),
//                        (long) (car.getUpdateTime() * 1000));
//            }
//        } else {
        lastManeuver = new StraightManeuver(car.getLaneIndex(), car.getVelocity().length(), getDistance(car, roadNetwork.getClosestLane(car.getPosition()).getInnerPoints().get(0)), (long) (car.getUpdateTime() * 1000));
//        }
        plan.add(lastManeuver);
        while (lastManeuver.getEndTime() <= to) {
            lastManeuver = new StraightManeuver(lastManeuver);
            plan.add(lastManeuver);
        }

        return plan;
    }

    public CarManeuver getCurrentManeuver() {
        return currentManeuver;
    }


}
