package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.simulation.Simulation;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;
import java.awt.*;
import java.util.LinkedList;
import java.util.List;
import java.util.ArrayList;

/**
 * Created by martin on 9.7.14.
 */
public class RouteAgent extends Agent {
    ///
    private static final float WAYPOINT_DISTANCE = 3.0f;
    private static float MAX_SPEED = 20;

    private static final float WP_COUNT_CONST = 0.2f;


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
     * @return
     */
    protected List<Action> agentReact() {
        LinkedList<Action> actions = new LinkedList<Action>(); // list of actions for the simulator
        ArrayList<Point3f> points;  // list of points on the way, used to be able to set speed to the action later

        RoadObject me = sensor.senseCurrentState(); // my current state

        // Simulator did not send update yet
        if (me == null) {
            actions.add(new WPAction(id, 0d, getInitialPosition(), 0));
            return actions;
        }



        Point2f position2D = new Point2f(me.getPosition().getX(), me.getPosition().getY());

        List<Point2f> wps = new LinkedList<Point2f>();
        Point2f waypoint = null;

        //int wpCount = Math.max(3, (int) (me.getVelocity().length() * WP_COUNT_CONST));
        //testing
        int wpCount = (int)me.getVelocity().length() +1; // how many waypoints before me will be calculated.
        points =  new ArrayList<Point3f>();
        navigator.setCheckpoint();

        //try to advance navigator closer to the actual position
        int maxMove = 10;  // how many points will be tried.
        while (maxMove-- > 0 && navigator.getRoutePoint().distance(position2D) > WAYPOINT_DISTANCE / 2) {
            navigator.advanceInRoute();
        }
        if( navigator.getRoutePoint().distance(position2D) > WAYPOINT_DISTANCE / 2){
            navigator.resetToCheckpoint();
        }else {
            navigator.setCheckpoint();
        }
        waypoint = navigator.getRoutePoint();

        float minSpeed = Float.MAX_VALUE; // minimal speed on the points before me
        for(int i=0;i<wpCount;i++)
        {
            // If the next waypoint is too close, go to the next in route
            while (waypoint.distance(navigator.getRoutePoint()) < WAYPOINT_DISTANCE){
                navigator.advanceInRoute();
            }
            waypoint = navigator.getRoutePoint();
            wps.add(waypoint);
            // vector from my position to the next waypoint
            Vector3f toNextPoint = new Vector3f(waypoint.x- me.getPosition().x,waypoint.y - me.getPosition().y,0);
            Vector3f velocity = me.getVelocity();
            float angle = velocity.angle(toNextPoint); // angle between my velocity and vector to the next point
            float speed;
            if(angle < 0.4) speed = MAX_SPEED; // if the curve is less than 20 degrees, go by the max speed.
            else if(angle > 6) speed = 2;    // minimal speed for curves.
            else
            {
                speed = 1/angle * 6;
            }
            if(speed < minSpeed) minSpeed = speed;  // all the next actions get the minimal speed.
            points.add(i,new Point3f(waypoint.x, waypoint.y, me.getPosition().z));
        }
        for(int i=0;i<wpCount;i++) // actual filling my outgoing actions
        {
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(),points.get(i),minSpeed));
        }
        navigator.resetToCheckpoint();

    return actions;
}
}
