package cz.agents.agentdrive.highway.agent;

import cz.agents.agentdrive.highway.environment.roadnet.ActualLanePosition;
import cz.agents.agentdrive.highway.environment.roadnet.Edge;
import cz.agents.agentdrive.highway.environment.roadnet.Junction;
import cz.agents.agentdrive.highway.environment.roadnet.Lane;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.maneuver.*;
import cz.agents.agentdrive.highway.storage.RoadObject;
import cz.agents.alite.configurator.Configurator;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import java.util.*;

public class GSDAgent extends SDAgent {

    protected static final Logger logger = Logger.getLogger(GSDAgent.class);
    private final static int CIRCLE_AROUND_FOR_SEARCH = 5;
    private final static int ANGLE_TO_JUNCTION = 60;

    private final static int DISTANCE_TO_THE_JUNCTION = Configurator.getParamInt("highway.safeDistanceAgent.distanceToActivateNM", 400);


    private int numberOfCollisions = 0;
    private boolean junctionMode = false;

    public GSDAgent(int id, RoadNetwork roadNetwork) {
        super(id, roadNetwork);
        logger.setLevel(Level.DEBUG);

    }

    public CarManeuver plan() {
        CarManeuver maneuver = null;
        RoadObject currState = sensor.senseCurrentState();
        // Simulator did not send update yet
        if (currState == null) {
            return null;
        }
        if (navigator.getLane() == null) {
            return null;
        }
        logger.debug("Startnode: " + currState);
        HighwaySituation situationPrediction = (HighwaySituation) getStatespace(currState);
        logger.debug("SS: " + situationPrediction);
        logger.debug("Situation: " + situationPrediction);
        logger.debug("Navigator: " + navigator.getRoutePoint());
        //HighwaySituation situationPrediction = (HighwaySituation) getStatespace(currState);
        int lane = currState.getLaneIndex();
        double velocity = currState.getVelocity().length();
        long updateTime = (long) (currState.getUpdateTime() * 1000);
        // I am point zero so distance to me is 0
        CarManeuver acc = new AccelerationManeuver(lane, velocity, 0, updateTime);
        CarManeuver str = new StraightManeuver(lane, velocity, 0, updateTime);
        CarManeuver left = new LaneLeftManeuver(lane, velocity, 0, updateTime);
        CarManeuver right = new LaneRightManeuver(lane, velocity, 0, updateTime);
        CarManeuver dec = new DeaccelerationManeuver(lane, velocity, 0, updateTime);
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

        logger.info("Planned maneuver for carID " + currState.getId() + " " + maneuver);

        return maneuver;
    }


    /**
     * calculation of the safe distance.
     */

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


    /**
     * This method gets vehicles around and generates the HighwaySituation that is used for reasoning.
     *
     * @param state This is the RoadObject of the operating vehicles.
     * @param cars  Vehicles nearby.
     * @param from  Starting time
     * @param to    Ending time.
     * @return HighwaySituation
     */
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
        ActualLanePosition temp = myActualLanePosition;
        myActualLanePosition = roadNetwork.getActualPosition(state.getPosition());
        if (!checkCorrectRoute()) {
            myActualLanePosition = temp;
        }
        navigator.setActualPosition(myActualLanePosition);

        //  checkCorrectRoute();
        //System.out.println(navigator.getRoute().contains());
        //  System.out.println(navigator.getLane().getInnerPoints());
        Lane myLane = myActualLanePosition.getLane();
        Edge myEdge = myActualLanePosition.getEdge();

        logger.debug("myLane = " + myLane.getIndex());
        num_of_lines = myEdge.getLanes().size();
        int myIndexOnRoute = myActualLanePosition.getIndex();//   getNearestWaipointIndex(state,myLane);

        //removing too far cars and myself from the collection
        for (RoadObject entry : cars) {
            float distanceToSecondCar = entry.getPosition().distance(state.getPosition());
            if (distanceToSecondCar > CHECKING_DISTANCE || state.getPosition().equals(entry.getPosition())) {
                continue;
            } else {
                if (distanceToSecondCar < 2.24)
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
        boolean nearTheJunction = (Utils.convertPoint3ftoPoint2f(state.getPosition()).distance(junctionwaypoint) < DISTANCE_TO_THE_JUNCTION && myNearestJunction.getIncLanes().size() > 1);
        //distance from the junction, should be determined by max allowed speed on the lane.
        for (RoadObject entry : nearCars) {
            Point2f intersectionWaypoint = junctionwaypoint;
            ArrayList<CarManeuver> predictedManeuvers;
            entryActualLanePosition = roadNetwork.getActualPosition(entry.getPosition());
            entryLane = entryActualLanePosition.getLane();
            if (myNearestJunction.equals(roadNetwork.getJunctions().get(entryLane.getParentEdge().getTo()))) {
                // if operating vehicle and other vehicle is heading to the same junction
                if (nearTheJunction) {
                    junctionMode = true;

                    //This part of code requires the knowledge of the long-term plan of the other vehicle.
                    // It determines if the vehicles croses their paths at the junction.
                    intersectionWaypoint = actualiseIntersectionWaypoint(entry, myLane, entryLane, junctionwaypoint);
                    if (intersectionWaypoint == null) continue;
                    // Transformation from the 2D space into the 1D space. Vehicles are virtually put on the one line
                    // to the junction by their distance to the junction.
                    float myDistance = getDistanceToTheJunction(myIndexOnRoute, myLane, intersectionWaypoint);
                    float entryDistance = getDistanceToTheJunction(entryActualLanePosition.getIndex(), entryLane, intersectionWaypoint);
                    if (entryDistance < myDistance) {
                        StraightManeuver man = new StraightManeuver(entry.getId(), entry.getVelocity().length(), myDistance - entryDistance, (long) (entry.getUpdateTime() * 1000));
                        situationPrediction.trySetCarAheadManeuver(man);
                    } else if (entryDistance == myDistance) //solution when distances are the same.
                    {
                        Random rand = new Random();
                        if (rand.nextFloat() > 0.5) {
                            StraightManeuver man = new StraightManeuver(entry.getId(), entry.getVelocity().length(), myDistance - entryDistance, (long) (entry.getUpdateTime() * 1000));
                            situationPrediction.trySetCarAheadManeuver(man);
                        }
                    }
                } else {
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
                    if (planned.getId().equals(entryLane.getParentEdge().getId())) {
                        if (Utils.getDistanceBetweenTwoRoadObjects(state, myActualLanePosition, entry, entryActualLanePosition, remE) < CHECKING_DISTANCE) {
                            continue;
                        }
                        predictedManeuvers = getPlannedManeuvers(state, myActualLanePosition, entry, entryActualLanePosition, from, to, remE);
                        situationPrediction.addAll(predictedManeuvers);
                        CarManeuver man = predictedManeuvers.get(0);
                        //TODO Improve this part to allow 2 lanes throw junction
//                        if ((Math.abs(state.getLaneIndex() - entry.getLaneIndex()) <= 1)) {
                        situationPrediction.trySetCarAheadManeuver(man);
                        situationPrediction.trySetCarLeftAheadMan(man);
                        situationPrediction.trySetCarRightAheadMan(man);
//                        }
                    }
                }
                continue;
            }
        }
        return situationPrediction;
    }

    /**
     * Method for fining nearest waipoint on the Lane
     *
     * @param state  Vehicle's state
     * @param myLane Vehicle's lane.
     * @return index on the lane.
     */
    @Deprecated
    private int getNearestWaipointIndex(RoadObject state, Lane myLane) {
        int myIndexOnRoute = 0;
        while (Utils.distanceP2P2(myLane.getInnerPoints().get(myIndexOnRoute), new Point2f(state.getPosition().x, state.getPosition().y)) > CIRCLE_AROUND_FOR_SEARCH) // Magical value
        {
            myIndexOnRoute++;
            if (myLane.getInnerPoints().size() == myIndexOnRoute) {
                myIndexOnRoute--;
                break;
            }
        }
        while (Utils.distanceP2P2(myLane.getInnerPoints().get(myIndexOnRoute), new Point2f(state.getPosition().x, state.getPosition().y)) <= CIRCLE_AROUND_FOR_SEARCH) {
            myIndexOnRoute++;
            if (myLane.getInnerPoints().size() == myIndexOnRoute) {
                myIndexOnRoute--;
                break;
            }
        }
        if (myIndexOnRoute >= myLane.getInnerPoints().size())
            myIndexOnRoute = myLane.getInnerPoints().size() - 1;
        // logger.debug("nearest waipoint id " + myIndexOnRoute);
        return myIndexOnRoute;
    }

    /**
     * Method for finding nearest waypoint
     *
     * @param me     Road object of operating vehicle
     * @param myLane vehicle's lane
     * @return index on the lane.
     */
    public int getNearestWaypointCloseEnough(RoadObject me, Lane myLane) {
        Point2f myPosition = Utils.convertPoint3ftoPoint2f(me.getPosition());
        Point2f innerPoint = myLane.getInnerPoints().get(0);
        int i = 0;
        while ((!maneuverTranslator.pointCloseEnough(innerPoint, myPosition, Utils.convertVector3ftoVector2f(me.getVelocity())) && i < myLane.getInnerPoints().size())) {

            innerPoint = myLane.getInnerPoints().get(i);
            i++;

        }
        //TODO FIX THIS
        if (i >= myLane.getInnerPoints().size())
            i = myLane.getInnerPoints().size() - 1;
        return i;
    }


    /**
     * This method is for measuring number of collisons by HighwayStorage.
     *
     * @return number of vehicle's collisions
     */
    public int getNumberOfColisions() {
        return numberOfCollisions;
    }

    private float getDistanceToTheJunction(int nearest, Lane myLane, Point2f intersectionWaypoint) {
        float distance = 0;
        int maxSize = myLane.getInnerPoints().size();
        for (int i = nearest + 1; i < maxSize; i++) {
            distance += myLane.getInnerPoints().get(i - 1).distance(myLane.getInnerPoints().get(i));
        }
        return distance + myLane.getInnerPoints().get(myLane.getInnerPoints().size() - 1).distance(intersectionWaypoint);
    }

    private Point2f actualiseIntersectionWaypoint(RoadObject entry, Lane myLane, Lane entryLane, Point2f junctionWaypoint) {
        Point2f intersectionWaypoint = junctionWaypoint;
        Map<Integer, Agent> agents = sensor.getAgents();
        GSDAgent entryAgent = (GSDAgent) agents.get(entry.getId());
        //calculation of optimised intersection point, if not found, other vehicle is ignored.
        if (!entryAgent.navigator.getFollowingEdgesInPlan().isEmpty() && !navigator.getFollowingEdgesInPlan().isEmpty()) {
            Edge entryNextEdge = entryAgent.navigator.getFollowingEdgesInPlan().iterator().next();
            Edge myNextEdge = navigator.getFollowingEdgesInPlan().iterator().next();

            Point2f p0 = myLane.getInnerPoints().get(myLane.getInnerPoints().size() - 1);
            Point2f p1 = myNextEdge.getLaneByIndex(0).getInnerPoints().get(0);
            Point2f p2 = entryLane.getInnerPoints().get(entryLane.getInnerPoints().size() - 1);
            Point2f p3 = entryNextEdge.getLaneByIndex(0).getInnerPoints().get(0);

            Point2f newCenter = Utils.isColision(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y);
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
}
