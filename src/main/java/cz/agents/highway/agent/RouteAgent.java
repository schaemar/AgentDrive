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
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

/**
 * Created by martin on 9.7.14.
 */
public class RouteAgent extends Agent {
    private int nextPoint = 0;
    private int nextEdge = 0;
    private List<Edge> route;
    ///
    private static final float CHANGE_RADIUS = 5.0f;

    public RouteAgent(int id) {
        super(id);

        initRoute(id);
    }

    /**
     * Generate list of edges from route definition
     * @param id Id of the vehicle
     */
    private void initRoute(int id) {
        Network network = Network.getInstance();
        XMLReader reader = XMLReader.getInstrance();
        Map<Integer, List<String>> routes = reader.getRoutes();
        Map<String, Edge> edges = network.getEdges();
        route = new LinkedList<Edge>();

        for (String edge: routes.get(id)) {
            route.add(edges.get(edge));
        }
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
    private Action agentReact() {
        Network network = Network.getInstance();
        RoadObject me = sensor.senseCurrentState();

        Point2f position2D = new Point2f(me.getPosition().getX(), me.getPosition().getY());
        // FIXME!!
        Lane lane = route.get(nextEdge).getLanes().get(String.format("%s_%d", route.get(nextEdge).getId(), me.getLane()));

        // If the next waypoint is too close, go to the next in route
        while (lane.getInnerPoints().get(nextPoint).distance(position2D) < CHANGE_RADIUS) {
            //TODO: do some check when lane ends
            if (nextPoint >= lane.getInnerPoints().size()-1) {
                nextPoint = 0;
                if (nextEdge >= route.size()-1) {
                    nextEdge = 0;
                } else {
                    nextEdge++;
                }
            } else {
                nextPoint++;
            }
            lane = route.get(nextEdge).getLanes().get(String.format("%s_%d", route.get(nextEdge).getId(), me.getLane()));
        }

        Point2f waypoint = lane.getInnerPoints().get(nextPoint);

        WPAction action = new WPAction(sensor.getId(), me.getUpdateTime(),
                new Point3f(waypoint.x, waypoint.y, me.getPosition().z), me.getVelocity().length());
        return action;
    }
}
