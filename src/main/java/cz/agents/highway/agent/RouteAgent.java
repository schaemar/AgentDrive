package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.environment.roadnet.XMLReader;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * Created by martin on 9.7.14.
 */
public class RouteAgent extends Agent {
    ///
    private static final float CHANGE_RADIUS = 5.0f;
    private static float MAX_SPEED = 30;

    private static final int TRY_COUNT = 10;

    /// Navigator generating route points
    private final RouteNavigator navigator;

    @Override
    public Point3f getInitialPosition() {

        //TODO  initial positioning with proper rotation
        Point2f p = navigator.getInitialPosition();
        return new Point3f(p.x,p.y,0);
    }

    public RouteAgent(int id) {
        super(id);
        navigator = new RouteNavigator(id);
    }


    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        this.sensor.registerReaction(new Reaction() {
            public void react(Event event) {
                if(event.getType().equals(HighwayEventType.UPDATED)){
                    actuator.act(agentReact());
                }
            }
        });
    }

    /**
     * Generate an action as a reaction
     * @return
     */
    protected Action agentReact() {
        Network network = Network.getInstance();
        RoadObject me = sensor.senseCurrentState();

        // Simulator did not send update yet
        if (me == null) {
            return new WPAction(id, 0d, getInitialPosition(), 0);
        }

        Point2f position2D = new Point2f(me.getPosition().getX(), me.getPosition().getY());

        // If the next waypoint is too close, go to the next in route
        Point2f waypoint = null;
        int i = 0;
        while (i < TRY_COUNT) {
            waypoint = navigator.getRoutePoint();
            if (waypoint.distance(position2D) < CHANGE_RADIUS) {
                navigator.advanceInRoute();
            }
            i++;
        }

        WPAction action = new WPAction(sensor.getId(), me.getUpdateTime(),
                new Point3f(waypoint.x, waypoint.y, me.getPosition().z), MAX_SPEED);
        return action;
    }
}
