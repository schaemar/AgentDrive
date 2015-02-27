package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.highway.maneuver.*;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.ManeuverAction;
import cz.agents.highway.storage.plan.WPAction;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector2f;
import javax.vecmath.Vector3f;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by martin on 9.7.14.
 */
public class RouteAgent extends Agent {
    ///
    private static final float CIRCLE_AROUND = 3.0f;  // Does not exactly correspond to the actual wayPoint distance, used to make circle around the car
    private static float MAX_SPEED = 20;

    private static final float WP_COUNT_CONST = 0.2f;
    private double lastUpateTime;
    private static int RIGHT = -1;
    private static int LEFT = 1;
    private static final double RADIUS = 1f;
    private static final double MAX_ANGLE = Math.PI / 2;
    private static final float EPSILON = 0.01f;


    @Override
    public Point3f getInitialPosition() {

        //TODO  initial positioning with proper rotation
        Point2f p = navigator.getInitialPosition();
        return new Point3f(p.x, p.y, 0);
    }

    public RouteAgent(int id) {
        super(id);
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

    /**
     * Generate an action as a reaction
     * Rout agent is adjusting speed according to the degrees of the curves and how many waypoints before himself will calculate.
     *
     * @return
     */
    protected List<Action> agentReact() {
        return generateWaypointInLane();
    }

    protected List<Action> agentReact(CarManeuver maneuver) {
        return translate(maneuver);
    }

    public List<Action> translate(CarManeuver maneuver) {
        if (maneuver == null) {
            LinkedList<Action> actions = new LinkedList<Action>();
            if(navigator.getLane() == null) {
                actions.add(new WPAction(id, 0d, new Point3f(0, 0, 0), -1));
                return actions;
            }
            Point2f initial = navigator.getInitialPosition();
            actions.add(new WPAction(id, 0d, new Point3f(initial.x, initial.y, 0), 0));

            return actions;
        }
        RoadObject me = sensor.senseCurrentState();
        // Check the type of maneuver
        if ((maneuver instanceof StraightManeuver) || (maneuver instanceof AccelerationManeuver)
                || (maneuver instanceof DeaccelerationManeuver)) {
            return generateWaypointInLane(0, maneuver);
        } else if (maneuver instanceof LaneLeftManeuver) {
            return generateWaypointInLane(/*me.getLaneIndex() + 1*/ LEFT, maneuver);
        } else if (maneuver instanceof LaneRightManeuver) {
            return generateWaypointInLane(/*me.getLaneIndex() - 1*/ RIGHT, maneuver);
        } else {
            LinkedList<Action> actions = new LinkedList<Action>();
            ManeuverAction res = new ManeuverAction(sensor.getId(), maneuver.getStartTime() / 1000.0,
                    maneuver.getVelocityOut(), maneuver.getLaneOut(), maneuver.getDuration());
            actions.add(res);
            return actions;
        }
    }

    //TODO Code duplicate with route agent
    //TODO use point close enough method from original Maneuver Translator
    private List<Action> generateWaypointInLane(int relativeLane, CarManeuver maneuver) {
        RoadObject me = sensor.senseCurrentState();
        LinkedList<Action> actions = new LinkedList<Action>();

        ArrayList<Point3f> points;  // list of points on the way, used to be able to set speed to the action later

        int wpCount = (int) me.getVelocity().length() + 1; // how many waypoints before me will be calculated.
        points = new ArrayList<Point3f>();
        navigator.setCheckpoint();

        Point2f position2D = new Point2f(me.getPosition().getX(), me.getPosition().getY());

        List<Point2f> wps = new LinkedList<Point2f>();
        Point2f waypoint = null;

        //try to advance navigator closer to the actual position
        int maxMove = 10;  // how many points will be tried.
        //how many waiponts ahead will be chcecked depending on the update time
        maxMove = (int) (((me.getUpdateTime() - lastUpateTime) * MAX_SPEED) / 1000) + 5;
        if (maxMove < 10) maxMove = 10;
        String uniqueIndex = navigator.getUniqueLaneIndex();
        // finding the nearest wayipont, if changing lane, set the first of the new lane.
        while (maxMove-- > 0 && navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.advanceInRoute();
        }
        // finding the nearest waipoint in the new lane.
        if (!navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            float initialPos = position2D.distance(navigator.getRoutePoint());
            do {
                navigator.advanceInRoute();
            } while (position2D.distance(navigator.getRoutePoint()) < initialPos);
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.setCheckpoint();

        }

        // waipoint not found, reset back
        if (navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.resetToCheckpoint();
        } else {
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.setCheckpoint();
        }


        if (relativeLane == RIGHT) {
            navigator.changeLaneRight();
            navigator.setCheckpoint();
        } else if (relativeLane == LEFT) {
            navigator.changeLaneLeft();
            navigator.setCheckpoint();
        }
        waypoint = navigator.getRoutePoint();


        float minSpeed = Float.MAX_VALUE; // minimal speed on the points before me
        //TODO fix than distance of waipoints is different than 1
        for (int i = 0; i <= maneuver.getPositionOut() || i < wpCount; i++) {
            // move 3 waipoints ahead
            while (waypoint.distance(navigator.getRoutePoint()) < CIRCLE_AROUND) {
                boolean myPlanEnding = navigator.advanceInRoute();
                if(!myPlanEnding)
                {
                    actions = new LinkedList<Action>();
                    Point2f initial = navigator.getInitialPosition();
                    actions.add(new WPAction(id, 0d, new Point3f(initial.x, initial.y, 0), -1));
                    return actions;
                }
            }
            waypoint = navigator.getRoutePoint();
            wps.add(waypoint);
            // vector from my position to the next waypoint
            Vector3f toNextPoint = new Vector3f(waypoint.x - me.getPosition().x, waypoint.y - me.getPosition().y, 0);
            Vector3f velocity = me.getVelocity();
            float angle = velocity.angle(toNextPoint); // angle between my velocity and vector to the next point
            float speed;
            if (Float.isNaN(angle)) {
                speed = 1;
            } else {
                if (angle < 0.4) speed = MAX_SPEED; // if the curve is less than 20 degrees, go by the max speed.
                else if (angle > 6) speed = 2;    // minimal speed for curves.
                else {
                    speed = 1 / angle * 6;
                }
            }
            if (speed < minSpeed) minSpeed = speed;  // all the next actions get the minimal speed.
            points.add(i, new Point3f(waypoint.x, waypoint.y, me.getPosition().z));
        }
        if (minSpeed > maneuver.getVelocityOut()) {
            minSpeed = (float) maneuver.getVelocityOut();
        }
        float speedChangeConst = (me.getVelocity().length() - minSpeed) / wpCount;
        for (int i = 0; i < wpCount; i++) // actual filling my outgoing actions
        {
            //scaling speed to the lowest
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), me.getVelocity().length() - (i + 1) * speedChangeConst));
        }
      /*
      only minimal speed set
      for(int i=0;i<=maneuver.getVelocityOut() && i<wpCount;i++)
        {
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(),points.get(i),minSpeed));
        }*/


        navigator.resetToCheckpoint();
        lastUpateTime = me.getUpdateTime();
        return actions;

    }

    private List<Action> generateWaypointInLane() {
        RoadObject me = sensor.senseCurrentState();
        LinkedList<Action> actions = new LinkedList<Action>();

        ArrayList<Point3f> points;  // list of points on the way, used to be able to set speed to the action later

        int wpCount = (int) me.getVelocity().length() + 1; // how many waypoints before me will be calculated.
        points = new ArrayList<Point3f>();
        navigator.setCheckpoint();

        Point2f position2D = new Point2f(me.getPosition().getX(), me.getPosition().getY());

        List<Point2f> wps = new LinkedList<Point2f>();
        Point2f waypoint = null;

        //try to advance navigator closer to the actual position
        int maxMove = 10;  // how many points will be tried.
        //how many waiponts ahead will be chcecked depending on the update time
        maxMove = (int) (((me.getUpdateTime() - lastUpateTime) * MAX_SPEED) / 1000) + 5;
        if (maxMove < 10) maxMove = 10;
        String uniqueIndex = navigator.getUniqueLaneIndex();
        // finding the nearest wayipont, if changing lane, set the first of the new lane.
        while (maxMove-- > 0 && navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.advanceInRoute();
        }
        // finding the nearest waipoint in the new lane.
        if (!navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            float initialPos = position2D.distance(navigator.getRoutePoint());
            do {
                navigator.advanceInRoute();
            } while (position2D.distance(navigator.getRoutePoint()) < initialPos);
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.setCheckpoint();

        }

        // waipoint not found, reset back
        if (navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.resetToCheckpoint();
        } else {
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.setCheckpoint();
        }
        waypoint = navigator.getRoutePoint();


        float minSpeed = Float.MAX_VALUE; // minimal speed on the points before me
        //TODO fix than distance of waipoints is different than 1
        for (int i = 0; i < wpCount; i++) {
            // move 3 waipoints ahead
            while (waypoint.distance(navigator.getRoutePoint()) < CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            waypoint = navigator.getRoutePoint();
            wps.add(waypoint);
            // vector from my position to the next waypoint
            Vector3f toNextPoint = new Vector3f(waypoint.x - me.getPosition().x, waypoint.y - me.getPosition().y, 0);
            Vector3f velocity = me.getVelocity();
            float angle = velocity.angle(toNextPoint); // angle between my velocity and vector to the next point
            float speed;
            if (Float.isNaN(angle)) {
                speed = 1;
            } else {
                if (angle < 0.4) speed = MAX_SPEED; // if the curve is less than 20 degrees, go by the max speed.
                else if (angle > 6) speed = 2;    // minimal speed for curves.
                else {
                    speed = 1 / angle * 6;
                }
            }
            if (speed < minSpeed) minSpeed = speed;  // all the next actions get the minimal speed.
            points.add(i, new Point3f(waypoint.x, waypoint.y, me.getPosition().z));
        }
        float speedChangeConst = (me.getVelocity().length() - minSpeed) / wpCount;
        for (int i = 0; i < wpCount; i++) // actual filling my outgoing actions
        {
            //scaling speed to the lowest
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(), points.get(i), me.getVelocity().length() - (i + 1) * speedChangeConst));
        }
        navigator.resetToCheckpoint();
        lastUpateTime = me.getUpdateTime();
        return actions;

    }

    private WPAction point2Waypoint(Point2f point, CarManeuver maneuver) {
        return new WPAction(sensor.getId(), maneuver.getStartTime() / 1000,
                new Point3f(point.x, point.y, sensor.senseCurrentState().getPosition().z),
                maneuver.getVelocityOut());
    }

    /**
     * This method determines whether the waypoint candidate is close enough (in radius) to the position
     * in the direction given by velocity vector
     *
     * @param innerPoint Waypoint candidate
     * @param position   Position
     * @param velocity   Velocity vector
     * @return
     */
    public boolean pointCloseEnough(Point2f innerPoint, Point2f position, Vector2f velocity) {
        // Direction vector of waypoint candidate relative to position
        Vector2f direction = new Vector2f();
        direction.sub(innerPoint, position);

        if (velocity.x == 0 && velocity.y == 0) {
            return innerPoint.distance(position) < 3;
        } else {
            return velocity.angle(direction) < MAX_ANGLE &&
                    distance(innerPoint, position, direction, velocity) < RADIUS;
        }
    }

    /**
     * This method computes the distance of the waypoint. It is the Euklidian distance of the waypoint
     * multiplied by absolute value of the sin of the angle between the direction of the waypoint
     * and the vector of velocity. This ensures, that waypoints that are less deviating from
     * the direction of the vehicle's movement and are close enough are picked.
     */
    private float distance(Point2f innerPoint, Point2f position, Vector2f direction, Vector2f velocity) {
        float d = innerPoint.distance(position);
        return d * d * Math.abs((float) Math.sin(direction.angle(velocity)) + EPSILON);
    }
}
