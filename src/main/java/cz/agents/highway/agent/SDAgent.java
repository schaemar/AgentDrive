package cz.agents.highway.agent;

import java.util.ArrayList;
import java.util.Collection;

import javax.vecmath.Point2d;
import javax.vecmath.Point3f;

import cz.agents.highway.protobuf.generated.simplan.PlanMessage;
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

public class SDAgent extends Agent {

	private static final Logger logger = Logger.getLogger(SDAgent.class);

    private final static double DISTANCE_TO_ACTIVATE_NM = Configurator.getParamDouble("highway.safeDistanceAgent.distanceToActivateNM",  300.0  );
    private final static double SAFETY_RESERVE          = Configurator.getParamDouble("highway.safeDistanceAgent.safetyReserveDistance",   4.0  );
    private final static double MAX_SPEED               = Configurator.getParamDouble("highway.safeDistanceAgent.maneuvers.maximalSpeed", 20.0  );
    private final static double MAX_SPEED_VARIANCE      = Configurator.getParamDouble("highway.safeDistanceAgent.maneuvers.maxSpeedVariance", 0.8 );

    private final static double LANE_SPEED_RATIO        = Configurator.getParamDouble("highway.safeDistanceAgent.laneSpeedRatio",            0.1);
    private final static long   PLANNING_TIME           = 1000;
    //FIXME: Determine number of lanes based on agent's current position
    private static final int    NUM_OF_LANES            =    1;

    private CarManeuver currentManeuver = null;
    private final ManeuverTranslator maneuverTranslator;

    // maximal speed after variance application
    private final double initialMaximalSpeed = (RandomProvider.getRandom().nextDouble() - 0.5) * 2 *MAX_SPEED_VARIANCE * MAX_SPEED  + MAX_SPEED;
    private double maximalSpeed = initialMaximalSpeed;

    public Action agentReact() {
        //return man2Action(plan());
        return maneuverTranslator.translate(plan());
    }

    private Action man2Action(CarManeuver man) {
        if(man ==null){
            return new WPAction(id, 0d, getInitialPosition(), 0);
        }
        return new ManeuverAction(sensor.getId(), man.getStartTime() / 1000.0, man.getVelocityOut(), man.getLaneOut(), man.getDuration());
    }

    public SDAgent(int id) {
        super(id);
        maneuverTranslator = new ManeuverTranslator(id);
    }

    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        maneuverTranslator.setSensor(sensor);
        this.sensor.registerReaction(new Reaction() {
            public void react(Event event) {
                if(event.getType().equals(HighwayEventType.UPDATED)){
                    actuator.act(agentReact());
                }
            }
        });
    }

    public void predictNext(HighwaySituation situationPrediction, RoadObject car, long predictionEndTime) {
        CarManeuver man = new StraightManeuver(car.getLane(), car.getVelocity().length(), getDistance(car), (long) (car.getUpdateTime() * 1000));
        if (!isCollision(man, situationPrediction)) {
            situationPrediction.add(man);
        }
    }

    public CarManeuver plan() {
        CarManeuver maneuver = null;
        RoadObject currState = sensor.senseCurrentState();
        // Simulator did not send update yet
        if (currState == null) {
            return null;
        }

        logger.debug("Startnode: " + currState);
        HighwaySituation situationPrediction = (HighwaySituation) getStatespace(currState);
        logger.debug("Situation: " + situationPrediction);

        int lane = currState.getLane();
        double velocity = currState.getVelocity().length();
        double distance = transGeoToDistance(currState.getPosition());
        long updateTime = (long) (currState.getUpdateTime() * 1000);

        CarManeuver acc   = new AccelerationManeuver  (lane, velocity, distance, updateTime);
        CarManeuver str   = new StraightManeuver      (lane, velocity, distance, updateTime);
        CarManeuver left  = new LaneLeftManeuver      (lane, velocity, distance, updateTime);
        CarManeuver right = new LaneRightManeuver     (lane, velocity, distance, updateTime);
        CarManeuver dec   = new DeaccelerationManeuver(lane, velocity, distance, updateTime);

        int preferredLane = getPreferredLane(currState);
        logger.debug("PreferredLane: " + preferredLane);
        if (isNarrowingMode(currState)) {
            logger.debug("Narrowing mode activated");
            CarManeuver preferredMan = null;
            if (preferredLane < currState.getLane()) {
                preferredMan = right;
            } else if (preferredLane > currState.getLane()) {
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
            if(currentManeuver !=null && currentManeuver.getClass().equals(LaneLeftManeuver.class) && isSafeMan(currState, left, situationPrediction)) {
                maneuver = left;
            }
            else if(currentManeuver !=null && currentManeuver.getClass().equals(LaneRightManeuver.class) && isSafeMan(currState, right, situationPrediction)) {
                maneuver = right;
            }
            else{

                if (isSafeMan(currState, right, situationPrediction)) {
                    maneuver = right;
                }
                else if (isSafeMan(currState, acc, situationPrediction)) {
                    maneuver = acc;
                } else if (isSafeMan(currState, str, situationPrediction)) {
                    maneuver = str;
                }else  if (isSafeMan(currState, left, situationPrediction)) {
                    maneuver = left;
                } else if (isSafeMan(currState, dec, situationPrediction)) {
                    maneuver = dec;
                } else {
                    logger.info("Nothing is safe, shouldnt happen!");
                    maneuver = dec;
                }
            }
        }
        logger.info("Planned maneuver: "+maneuver);
        currentManeuver = maneuver;
        return maneuver;
    }

    private double transGeoToDistance(double x, double y) {
        return sensor.getRoadDescription().distance(new Point2d(x, y));
    }
    private double transGeoToDistance(Point3f position) {
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
        if(Configurator.getParamBool("highway.safeDistanceAgent.narrowingModeActive", false).equals(false)) return false;

        int lane = state.getLane();
        double distance = getDistance(state);

        // following is universal
        boolean myLaneEnding = !isLaneGoingOn(distance, DISTANCE_TO_ACTIVATE_NM, lane);
        int lastLane = state.getLane() - 1;
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
            if (obs.getLane() != lane)
                continue;
            double obsDist = getDistance(obs);
            if (obsDist > position && obsDist < position + distance) {
                return false;
            }
        }
        return true;
    }

    private double getDistance(RoadObject state) {
        return transGeoToDistance(state.getPosition());
    }

    private boolean isSafeMan(RoadObject state, CarManeuver man, HighwaySituation situation) {
        boolean narrowingMode = isNarrowingMode(state);

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
        } else if (!isLaneGoingOn(man.getPositionOut(), safeDistance(man, SAFETY_RESERVE), state.getLane())) {
            return true;
        } else
            return false;
    }

    private double safeDistance(CarManeuver man, double safetyReserve) {
        // TODO get the a0 from the car
        double a0 = -4;
        double v0 = man.getVelocityOut();
        double v1 = 0;
        return safeDistance(a0, v0, v1, safetyReserve);
    }

    private double safeDistance(CarManeuver manAhead, CarManeuver manBehind, double safetyReserve) {
        // TODO get the a0 from the car
        double a0 = -4;
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
    		logger.info("laneMasSpeed= "+laneMaxSpeed +" maximalSpeed="+maximalSpeed+" speedVariance="+MAX_SPEED_VARIANCE+ " MAX_SPEED conf = "+MAX_SPEED);
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
        if (laneOut > NUM_OF_LANES) {
            logger.debug("laneOut = " + laneOut);
            return false;
        }

        return true;
    }

    public HighwaySituation generateSS(RoadObject state, Collection<RoadObject> cars, long from, long to) {
        HighwaySituation situationPrediction = new HighwaySituation();
        logger.debug("GenerateSS:");

        for (RoadObject entry : cars) {

            ArrayList<CarManeuver> predictedManeuvers = getPlannedManeuvers(entry, from, to);
            situationPrediction.addAll(predictedManeuvers);

            CarManeuver man = predictedManeuvers.get(0);
            double otherCarPosition = man.getPositionIn();
            double thisCarPosition = getDistance(state);
            int thisCarLane = state.getLane();

            logger.debug("Other car " + entry.getId() + ": Maneuver: " + man);
            logger.debug("This  car: Distance: " + thisCarPosition);
            logger.debug("Other car: Distance: " + otherCarPosition);

            if (man.getLaneOut() == thisCarLane && otherCarPosition > thisCarPosition) {
                situationPrediction.trySetCarAheadManeuver(man);
            } else if (((man.getLaneOut() == thisCarLane + 1) || (man.getLaneOut() == thisCarLane + 2 && man
                    .getLaneIn() == thisCarLane + 1)) && man.getPositionIn() < thisCarPosition) {
                situationPrediction.trySetCarLeftMan(man);
            } else if (((man.getLaneOut() == thisCarLane - 1) || (man.getLaneOut() == thisCarLane - 2 && man
                    .getLaneIn() == thisCarLane - 1)) && man.getPositionIn() < thisCarPosition) {
                situationPrediction.trySetCarRightMan(man);
            } else if (((man.getLaneOut() == thisCarLane + 1) || ((man.getLaneOut() == thisCarLane || man
                    .getLaneOut() == thisCarLane + 2) && man.getLaneIn() == thisCarLane + 1))
                    && man.getPositionIn() > thisCarPosition) {
                situationPrediction.trySetCarLeftAheadMan(man);
            } else if (((man.getLaneOut() == thisCarLane - 1) || ((man.getLaneOut() == thisCarLane || man
                    .getLaneOut() == thisCarLane - 2) && man.getLaneIn() == thisCarLane - 1))
                    && man.getPositionIn() > thisCarPosition) {
                situationPrediction.trySetCarRightAheadMan(man);
            }

        }
        return situationPrediction;
    }

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
//            lastManeuver = new DeaccelerationManeuver(car.getLane(), car.getVelocity().length(), getDistance(car),
//                    (long) (car.getUpdateTime() * 1000));
//
//        } else if (turning != null) {
//            if (turning.equals("left")) {
//                lastManeuver = new LaneLeftManeuver(car.getLane(), car.getVelocity().length(), getDistance(car),
//                        (long) (car.getUpdateTime() * 1000));
//            } else {
//                lastManeuver = new LaneRightManeuver(car.getLane(), car.getVelocity().length(), getDistance(car),
//                        (long) (car.getUpdateTime() * 1000));
//            }
//        } else {
        lastManeuver = new StraightManeuver(car.getLane(), car.getVelocity().length(), getDistance(car), (long) (car.getUpdateTime() * 1000));
//        }
        plan.add(lastManeuver);
        while (lastManeuver.getEndTime() <= to) {
            lastManeuver = new StraightManeuver(lastManeuver);
            plan.add(lastManeuver);
        }

        return plan;
    }

}
