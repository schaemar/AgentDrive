package cz.agents.highway.agent;

import java.util.*;

import javax.vecmath.*;

import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Junction;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.protobuf.generated.simplan.VectorProto;
import cz.agents.highway.storage.plan.WPAction;
import org.apache.log4j.Logger;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.configurator.Configurator;
import cz.agents.highway.environment.RandomProvider;
import cz.agents.highway.maneuver.AccelerationManeuver;
import cz.agents.highway.maneuver.CarManeuver;
import cz.agents.highway.maneuver.DeaccelerationManeuver;
import cz.agents.highway.maneuver.HighwaySituation;
import cz.agents.highway.maneuver.LaneLeftManeuver;
import cz.agents.highway.maneuver.LaneRightManeuver;
import cz.agents.highway.maneuver.StraightManeuver;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.ManeuverAction;

public class GSDAgent extends RouteAgent {

    protected static final Logger logger = Logger.getLogger(SDAgent.class);

    //private final static double DISTANCE_TO_ACTIVATE_NM = Configurator.getParamDouble("highway.safeDistanceAgent.distanceToActivateNM",  300.0  );
    private final static double SAFETY_RESERVE          = Configurator.getParamDouble("highway.safeDistanceAgent.safetyReserveDistance",   4.0  );
    private final static double MAX_SPEED               = Configurator.getParamDouble("highway.safeDistanceAgent.maneuvers.maximalSpeed", 70.0  );
    private final static double MAX_SPEED_VARIANCE      = Configurator.getParamDouble("highway.safeDistanceAgent.maneuvers.maxSpeedVariance", 0.8 );

    private final static double LANE_SPEED_RATIO        = Configurator.getParamDouble("highway.safeDistanceAgent.laneSpeedRatio",            0.1);
    private final static long   PLANNING_TIME           = 1000;
    private final static int CIRCLE_AROUND_FOR_SEARCH = 5;
    private final static int ANGLE_TO_JUNCTION = 60;

    private final static int DISTANCE_TO_THE_JUNCTION = Configurator.getParamInt("highway.safeDistanceAgent.distanceToActivateNM", 400);
    private final static int CHECKING_DISTANCE = 500;
    //FIXME: Determine number of lanes based on agent's current position
    protected int num_of_lines;

    private int numberOfCollisions=0;
    private CarManeuver currentManeuver = null;

    // maximal speed after variance application
    private final double initialMaximalSpeed = (RandomProvider.getRandom().nextDouble() - 0.5) * 2 *MAX_SPEED_VARIANCE * MAX_SPEED  + MAX_SPEED;
    private double maximalSpeed = initialMaximalSpeed;

    private HighwayEnvironment highwayEnvironment;
    private Edge myEdge;
    private Lane myLane;
    @Override
    public List<Action> agentReact() {
        return super.agentReact(plan());
    }

    private Action man2Action(CarManeuver man) {
        if(man ==null){
            return new WPAction(id, 0d, getInitialPosition(), 0);
        }
        return new ManeuverAction(sensor.getId(), man.getStartTime() / 1000.0, man.getVelocityOut(), man.getLaneOut(), man.getDuration());
    }

    public GSDAgent(int id, HighwayEnvironment hgw) { // TODO don't use whole HighwayEnviroment in agent
        super(id);
        num_of_lines = 1;
        this.highwayEnvironment = hgw;
    }
    /*
    public void predictNext(HighwaySituation situationPrediction, RoadObject car, long predictionEndTime) {
        CarManeuver man = new StraightManeuver(car.getLaneIndex(), car.getVelocity().length(), getDistance(car), (long) (car.getUpdateTime() * 1000));
        if (!isCollision(man, situationPrediction)) {
            situationPrediction.add(man);
        }
    }*/
    long time = -1;
    boolean test = true;
    public CarManeuver plan() {
        CarManeuver maneuver = null;
        RoadObject currState = sensor.senseCurrentState();
        // Simulator did not send update yet
        if (currState == null) {
            return null;
        }
        if(navigator.getLane() == null)
        {
            logger.info("My id is " + this.id + " number of colisions is " + numberOfCollisions);

          /*  highwayEnvironment.getStorage().removeAgent(this.id);*/
            highwayEnvironment.getStorage().getPosCurr().remove(this.id);
            return null;
        }
       // logger.debug("Startnode: " + currState);
        HighwaySituation situationPrediction = (HighwaySituation) getStatespace(currState);
      //  logger.debug("Situation: " + situationPrediction);

        int lane = currState.getLaneIndex();
        double velocity = currState.getVelocity().length();
        long updateTime = (long) (currState.getUpdateTime() * 1000);
        // I am point zero so distance to me is 0
        CarManeuver acc   = new AccelerationManeuver  (lane, velocity, 0, updateTime);
        CarManeuver str   = new StraightManeuver      (lane, velocity, 0, updateTime);
        CarManeuver left  = new LaneLeftManeuver      (lane, velocity, 0, updateTime);
        CarManeuver right = new LaneRightManeuver     (lane, velocity, 0, updateTime);
        CarManeuver dec   = new DeaccelerationManeuver(lane, velocity, 0, updateTime);

        int preferredLane = 0;
       // logger.debug("PreferredLane: " + preferredLane);

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
                logger.info("Nothing is safe, shouldnt happen!");
                maneuver = dec;
            }
        }
        //testing scenario
        /*if(currState.getId() == 0 && navigator.getActualPointer() > 5)
        {
            maneuver = dec;
        }*/
/*
        if(currState.getId() == 1 && navigator.getActualPointer() > 140 && test)
        {
            maneuver = dec;
            if(time==-1) time= System.currentTimeMillis();
            if(System.currentTimeMillis() - time > 50000)
            {
                test = false;
            }
        }*/

      //  logger.info("Planned maneuver for carID " + currState.getId() + " " +maneuver);
        currentManeuver = maneuver;
        return maneuver;
    }

    private double transGeoToDistance(double x, double y) {
        return sensor.getRoadDescription().distance(new Point2d(x, y));
    }
    @Deprecated
    protected double transGeoToDistance(Point3f position) {
        return transGeoToDistance(position.x, position.y);
    }
    @Deprecated
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
        // return transGeoToDistance(state.getPosition());
        return 0;
    }

    private boolean isSafeMan(RoadObject state, CarManeuver man, HighwaySituation situation) {
        if (isHighwayCollision(state, man)) {
            if(id ==2) logger.info("Highway Collision detected!" + man);
            return false;
        }
        if (isRulesCollision(man)) {
            if(id ==2) logger.info("Rules Collision detected! " + man);
            return false;

        }
        HighwaySituation emptySituation = new HighwaySituation();
        emptySituation.clear();
        if (!isSafe(man, emptySituation))
            return false;

        if (man.getClass().equals(AccelerationManeuver.class)
                || man.getClass().equals(StraightManeuver.class)) {

            return isInSafeDistance(situation.getCarAheadMan(), man);// sufficient if not narrowing
        } else if (man.getClass().equals(LaneLeftManeuver.class)) {
            if(id ==2) logger.info("LEFT_MAN_OUTPUT: " + isInSafeDistance(situation.getCarLeftAheadMan(), man)
                    + " " + isInSafeDistance(man, situation.getCarLeftMan()));
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
            // NOT USED, code for obstacles on the road
        } /*else if (!isLaneGoingOn(man.getPositionOut(), safeDistance(man, SAFETY_RESERVE), state.getLaneIndex())) {
            return true;
        }*/ else
            return false;
    }
    @Deprecated
    private double safeDistance(CarManeuver man, double safetyReserve) {
        // TODO get the a0 from the car
        double a0 = -4;
        double v0 = man.getVelocityOut();
        double v1 = 0;
        return safeDistance(a0, v0, v1, safetyReserve);
    }

    private double safeDistance(CarManeuver manAhead, CarManeuver manBehind, double safetyReserve) {
        // TODO get the a0 from the car
        // TODO Discover what the hell is a0
        double a0 = -1;
        double v0 = manBehind.getVelocityOut();
        double v1 = manAhead.getVelocityIn();
        double safeDist = safeDistance(a0, v0, v1, safetyReserve);
        return safeDist;
    }

    private double safeDistance(double a0, double v0, double v1) {
        double safeDist = (v1 * v1 - v0 * v0) / (2 * a0);
    //    double safeDist = v0*2;
        return safeDist;
    }

    private double safeDistance(double a0, double v0, double v1, double safetyReserve) {
        double safeDist = safeDistance(a0, v0, v1) + safetyReserve;
        return safeDist;
    }

    private boolean isInSafeDistance(CarManeuver manAhead, CarManeuver manBehind) {
        if (manAhead == null || manBehind == null) return true;
        double realDist = manAhead.getPositionIn() - manBehind.getPositionOut();
        // continousHighway condition
        // if(realDist<0)realDist+=carSensorManeuver.getHighwayLength();
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
        //if(carManeuver.getClass().equals(AccelerationManeuver.class) || carManeuver.getClass().equals(StraManeuver.class)){
        double laneMaxSpeed = maximalSpeed + carManeuver.getLaneIn() * LANE_SPEED_RATIO * maximalSpeed;
       // logger.info("laneMasSpeed= "+laneMaxSpeed +" maximalSpeed="+maximalSpeed+" speedVariance="+MAX_SPEED_VARIANCE+ " MAX_SPEED conf = "+MAX_SPEED);
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
          //  logger.debug("LaneOut= " + laneOut);
            return false;
        }
        if (laneOut >= num_of_lines) {
           // logger.debug("laneOut = " + laneOut);
            return false;
        }

        return true;
    }
    // Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
    // intersect the intersection point may be stored in the floats i_x and i_y.
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
    public HighwaySituation generateSS(RoadObject state, Collection<RoadObject> cars, long from, long to) {

        HighwaySituation situationPrediction = new HighwaySituation();

       // logger.debug("GenerateSS:");
        Collection<RoadObject> nearCars = new Vector<RoadObject>();
        //FAST SOLUTION
        myLane = navigator.getLane();
        //SLOW
       // myLane = highwayEnvironment.getRoadNetwork().getLane(state.getPosition());

        myEdge = myLane.getEdge();
        num_of_lines = myEdge.getLanes().size();
        int myIndexOnRoute=getNearestWaipointIndex(state,myLane);

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
        //adding my prediction to the collection
       // situationPrediction.addAll(getPlannedManeuvers(state, from, to));

        Lane entryLane =null;
        int numberOfLanes = myEdge.getLanes().size();
        Junction myNearestJunction = highwayEnvironment.getRoadNetwork().getJunctions().get(myEdge.getTo());

        //TODO use config for this
        Point2f junctionwaypoint = myNearestJunction.getCenter();
        boolean nearTheJunction = (convertPoint3ftoPoint2f(state.getPosition()).distance(junctionwaypoint) < DISTANCE_TO_THE_JUNCTION && myNearestJunction.getIncLanes().size() > 1);
        if(myNearestJunction.getId().equals("preparation")) nearTheJunction = false;
        //distance from the junction, should be determined by max allowed speed on the lane.
        for(RoadObject entry : nearCars) {


            Point2f intersectionWaypoint = junctionwaypoint;
            ArrayList<CarManeuver> predictedManeuvers;
            entryLane = highwayEnvironment.getRoadNetwork().getLane(entry.getPosition());
            //TODO Fix this if else structur
            if(myNearestJunction.equals(highwayEnvironment.getRoadNetwork().getJunctions().get(entryLane.getEdge().getTo()))) {
                if (nearTheJunction)
                {

                    //TODO maybe better implementation that does not use highway storage
                    //TODO Optimilize to run this once
                    Map<Integer, Agent> agents = highwayEnvironment.getStorage().getAgents();
                    GSDAgent entryAgent = (GSDAgent)agents.get(entry.getId());
                    if(!entryAgent.navigator.getFollowingEdgesInPlan().isEmpty() && !navigator.getFollowingEdgesInPlan().isEmpty()) {
                        Edge entryNextEdge = entryAgent.navigator.getFollowingEdgesInPlan().iterator().next();
                        Edge myNextEdge = navigator.getFollowingEdgesInPlan().iterator().next();

                        Point2f p0 = myLane.getInnerPoints().get(myLane.getInnerPoints().size() - 1);
                        Point2f p1 = myNextEdge.getLaneByIndex(0).getInnerPoints().get(0);
                        Point2f p2 = entryLane.getInnerPoints().get(entryLane.getInnerPoints().size() - 1);
                        Point2f p3 = entryNextEdge.getLaneByIndex(0).getInnerPoints().get(0);

                        Point2f newCenter = isColision(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
                        if (newCenter == null && !myLane.equals(entryLane)) {
                            continue;
                        } else if (newCenter != null && !myLane.equals(entryLane)) {
                            //TODO fix this hotfix, probably in route agent.
                            if (junctionwaypoint.distance(newCenter) < 10) {
                                intersectionWaypoint = new Point2f((newCenter.x + junctionwaypoint.x) / 2, (newCenter.y + junctionwaypoint.y) / 2);
                            }
                            /*else {
                                System.out.println("Something is terribly wrong");
                            }*/
                        }
                    }

                    Point2f myPosition = convertPoint3ftoPoint2f(state.getPosition());
                    Point2f entryPosition = convertPoint3ftoPoint2f(entry.getPosition());

                    Vector2f toTheCentre = new Vector2f((intersectionWaypoint.x - convertPoint3ftoPoint2f(entry.getPosition()).x),
                            (intersectionWaypoint.y - convertPoint3ftoPoint2f(entry.getPosition()).y));

                    double maxAngle = ANGLE_TO_JUNCTION * Math.PI / 180;    //Max used angle
                    double ange = convertVector3ftoVector2f(entry.getVelocity()).angle(toTheCentre);
                    if (ange > maxAngle) { //TODO Fix

                    } else {
                        double hypotenuse = entry.getVelocity().length();
                        double d = hypotenuse * Math.cos(ange);
                        if (Double.isNaN(d)) d = 0;
                        toTheCentre.normalize();
                        Vector2f resultant = new Vector2f((float) toTheCentre.x * (float) d, (float) toTheCentre.y * (float) d);
                        float myDistance = myPosition.distance(intersectionWaypoint);
                        float entryDistance = entryPosition.distance(intersectionWaypoint);
                        if (entryDistance < myDistance) {
                            //Možná bude třeba setnout i v pravo a vlevo aby auto neměnilo pruh
                            StraightManeuver man = new StraightManeuver(entry.getId(), resultant.length(), myDistance - entryDistance, (long) (entry.getUpdateTime() * 1000));
                            situationPrediction.trySetCarAheadManeuver(man);
                            situationPrediction.trySetCarRightAheadMan(man);
                            situationPrediction.trySetCarLeftAheadMan(man);
                        }
                        else if(entryDistance == myDistance) //TODO check this
                        {
                            Random rand = new Random();
                            if(rand.nextFloat() > 0.5)
                            {
                                StraightManeuver man = new StraightManeuver(entry.getId(), resultant.length(), myDistance - entryDistance, (long) (entry.getUpdateTime() * 1000));
                                situationPrediction.trySetCarAheadManeuver(man);
                                situationPrediction.trySetCarRightAheadMan(man);
                                situationPrediction.trySetCarLeftAheadMan(man);
                            }
                        }
                    }

                } else {
                    if (entryLane.getEdge().equals(myEdge)) {
                        predictedManeuvers = getPlannedManeuvers(state, myLane, entry, entryLane, from, to, null);
                        situationPrediction.addAll(predictedManeuvers);

                        CarManeuver man = predictedManeuvers.get(0);

                        int entryNearestWaipoint = getNearestWaipointIndex(entry, entryLane);
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
            if (!entryLane.getEdge().equals(myEdge)) {
                // TODO situation where car is in the different road
                List<Edge> rem = navigator.getFollowingEdgesInPlan();
              //  logger.info("Checking this entry " + entry);
                for (Edge planned : rem) {
                    if (planned.equals(entryLane.getEdge())) {
                        predictedManeuvers = getPlannedManeuvers(state, myLane, entry, entryLane, from, to, rem);
                        situationPrediction.addAll(predictedManeuvers);
                        CarManeuver man = predictedManeuvers.get(0);

                       /* if ((state.getLaneIndex() - entry.getLaneIndex()) == 1) {
                            //right
                            situationPrediction.trySetCarRightAheadMan(man);
                        } else if ((state.getLaneIndex() - entry.getLaneIndex()) == 1) {
                            // left
                            situationPrediction.trySetCarLeftAheadMan(man);
                        } else {
                            //same
                            situationPrediction.trySetCarAheadManeuver(man);
                        }
                        */
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
    private int getNearestWaipointIndex(RoadObject state,Lane myLane)
    {
        int myIndexOnRoute = 0;

        float test = distanceP2P2(myLane.getInnerPoints().get(myIndexOnRoute), new Point2f(state.getPosition().x, state.getPosition().y));
        while(distanceP2P2(myLane.getInnerPoints().get(myIndexOnRoute), new Point2f(state.getPosition().x, state.getPosition().y))  > CIRCLE_AROUND_FOR_SEARCH) // Magical value
        {
            myIndexOnRoute++;  //TODO fix this
            if(myLane.getInnerPoints().size() == myIndexOnRoute)
            {
                myIndexOnRoute--;
                break;
            }
        }
        while(distanceP2P2(myLane.getInnerPoints().get(myIndexOnRoute), new Point2f(state.getPosition().x, state.getPosition().y))  <= CIRCLE_AROUND_FOR_SEARCH)
        {
            myIndexOnRoute++;  //TODO fix this
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
    public float distanceP2P2(Point2f innerPoint, Point2f position) {
        return innerPoint.distance(position);

    }

    public ArrayList<CarManeuver> getPlannedManeuvers(RoadObject me,Lane myLane,RoadObject car,Lane hisLane, long from, long to,List<Edge> rem) {

        ArrayList<CarManeuver> plan = new ArrayList<CarManeuver>();
        // TODO add a part of plan that is between from and to
        CarManeuver lastManeuver;
        lastManeuver = new DeaccelerationManeuver(car.getLaneIndex(), car.getVelocity().length(), getDistanceBetweenTwoRoadObjects(me,myLane,car,hisLane,rem), (long) (car.getUpdateTime() * 1000));
        plan.add(lastManeuver);
        while (lastManeuver.getEndTime() <= to) {
            lastManeuver = new DeaccelerationManeuver(lastManeuver);
            plan.add(lastManeuver);
        }

        return plan;
    }
    @Deprecated
    public ArrayList<CarManeuver> getPlannedManeuvers(RoadObject car, long from, long to) {
        // if has no plan -> predict TODO use more sofisticated prediction - not
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
        lastManeuver = new StraightManeuver(car.getLaneIndex(), car.getVelocity().length(), 0, (long) (car.getUpdateTime() * 1000));
//        }
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
     * @param me
     * @param myLane
     * @param other
     * @param otherLane
     * @return
     */
    private double getDistanceBetweenTwoRoadObjects(RoadObject me,Lane myLane,RoadObject other,Lane otherLane,List<Edge> rem) {
        //two possibilities how to find nearest waipoint, point close enough more sophisticated but does not work always

        int nearestAa = getNearestWaipointIndex(me, myLane);
        int nearestBb = getNearestWaipointIndex(other, otherLane);

        int nearestA = getNearestWaipointCloseEnough(me, myLane);
        int nearestB = getNearestWaipointCloseEnough(other, otherLane);
        //only for debug
        Edge myEdg = myLane.getEdge();
        Edge his = otherLane.getEdge();
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
                Lane tem = rem.get(i).getLaneByIndex(0);
                for (int d = 1; d < tem.getInnerPoints().size();d++) {
                    distC += tem.getInnerPoints().get(d - 1).distance(tem.getInnerPoints().get(d));
                }

            }

            //TODO FIX THIS!!!
            return distC; // debug
        }
    }
    public int getNearestWaipointCloseEnough(RoadObject me,Lane myLane)
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
    public int getNumberOfColisions()
    {
        return numberOfCollisions;
    }
}
