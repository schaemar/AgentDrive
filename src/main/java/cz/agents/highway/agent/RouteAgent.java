package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import java.util.LinkedList;
import java.util.List;

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
     *
     * @return
     */
    protected List<Action> agentReact() {
        LinkedList<Action> actions = new LinkedList<Action>();
        RoadObject me = sensor.senseCurrentState();

        // Simulator did not send update yet
        if (me == null) {
            actions.add(new WPAction(id, 0d, getInitialPosition(), 0));
            return actions;
        }

        Point2f position2D = new Point2f(me.getPosition().getX(), me.getPosition().getY());

        List<Point2f> wps = new LinkedList<Point2f>();
        Point2f waypoint = null;

        int wpCount = Math.max(3, (int) (me.getVelocity().length() * WP_COUNT_CONST));
        navigator.setCheckpoint();

        //try to advance navigator closer to the actual position
        int a = 10;
        while (a-- > 0 && navigator.getRoutePoint().distance(position2D) > WAYPOINT_DISTANCE / 2) {
            navigator.advanceInRoute();
        }
        if( navigator.getRoutePoint().distance(position2D) > WAYPOINT_DISTANCE / 2){
            navigator.resetToCheckpoint();
        }else {
            navigator.setCheckpoint();
        }
        waypoint = navigator.getRoutePoint();

        for (int i = 0; i < wpCount; i++) {
            // If the next waypoint is too close, go to the next in route
            while (waypoint.distance(navigator.getRoutePoint()) < WAYPOINT_DISTANCE){
                navigator.advanceInRoute();
            }
            waypoint = navigator.getRoutePoint();
            wps.add(waypoint);
            actions.add(new WPAction(sensor.getId(), me.getUpdateTime(),
                    new Point3f(waypoint.x, waypoint.y, me.getPosition().z), MAX_SPEED));
        }
        navigator.resetToCheckpoint();


    return actions;
}
}
