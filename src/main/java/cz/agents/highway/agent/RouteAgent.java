package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;
import java.util.LinkedList;
import java.util.List;
import java.util.ArrayList;

/**
 * Created by martin on 9.7.14.
 */
public class RouteAgent extends Agent {
    ///
    private static final float CIRCLE_AROUND = 6.0f; // Does not exactly corespond to the actual waipoint distance, used to make circle around the car
    private static float MAX_SPEED = 20;

    private static final float WP_COUNT_CONST = 0.2f;
    private double lastUpateTime;


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

        int wpCount = (int)me.getVelocity().length() +1; // how many waypoints before me will be calculated.
        points =  new ArrayList<Point3f>();
        navigator.setCheckpoint();

        //try to advance navigator closer to the actual position
        int maxMove = 10;  // how many points will be tried.
        //how many waiponts ahead will be chcecked depending on the update time
        maxMove = (int)(((me.getUpdateTime() - lastUpateTime)*MAX_SPEED)/1000) + 5;
        if(maxMove < 10) maxMove = 10;
        String uniqueIndex = navigator.getUniqueLaneIndex();
        // finding the nearest wayipont, if changing lane, set the first of the new lane.
        while (maxMove-- > 0 && navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND&& navigator.getUniqueLaneIndex().equals(uniqueIndex)) {
            navigator.advanceInRoute();
        }
        // finding the nearest waipoint in the new lane.
        if(!navigator.getUniqueLaneIndex().equals(uniqueIndex))
        {
            float initialPos = position2D.distance(navigator.getRoutePoint());
            do{
                navigator.advanceInRoute();
            }while(position2D.distance(navigator.getRoutePoint())  < initialPos);

        }
        // waipoint not found, reset back
        if( navigator.getRoutePoint().distance(position2D) > CIRCLE_AROUND && navigator.getUniqueLaneIndex().equals(uniqueIndex)){
            navigator.resetToCheckpoint();
        }else {
            while (navigator.getRoutePoint().distance(position2D) <= CIRCLE_AROUND) {
                navigator.advanceInRoute();
            }
            navigator.advanceInRoute();
            navigator.advanceInRoute();
            navigator.advanceInRoute();
            navigator.advanceInRoute();
            navigator.advanceInRoute();
            navigator.advanceInRoute();

            navigator.setCheckpoint();
        }
        waypoint = navigator.getRoutePoint();

        float minSpeed = Float.MAX_VALUE; // minimal speed on the points before me
        for(int i=0;i<wpCount;i++)
        {
            // move 3 waipoints ahead
            while (waypoint.distance(navigator.getRoutePoint()) < CIRCLE_AROUND){
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
        float speedChangeConst = (me.getVelocity().length() - minSpeed)/wpCount;
        for(int i=0;i<wpCount;i++) // actual filling my outgoing actions
        {
            //scaling speed to the lowest
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(),points.get(i),me.getVelocity().length()-(i+1)*speedChangeConst));
        }
        navigator.resetToCheckpoint();
        lastUpateTime = me.getUpdateTime();
    return actions;
}
}
