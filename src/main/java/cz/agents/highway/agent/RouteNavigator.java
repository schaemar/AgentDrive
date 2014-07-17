package cz.agents.highway.agent;

import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.environment.roadnet.XMLReader;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Class used for car navigation on given route
 * Created by wmatex on 15.7.14.
 */
public class RouteNavigator {
    private final int id;

    private int nextEdge = 0;
    private int nextPoint = 0;

    /// Route represented as a list of edges, that the car should visit
    private final List<Edge> route = new ArrayList<Edge>();

    public RouteNavigator(int id) {
        this.id = id;
        initRoute(id);
    }

    /**
     * Generate list of edges from route definition
     * @param id Id of the vehicle
     */
    private void initRoute(int id) {
        Network network = Network.getInstance();
        XMLReader reader = XMLReader.getInstance();
        Map<Integer, List<String>> routes = reader.getRoutes();
        Map<String, Edge> edges = network.getEdges();

        for (String edge: routes.get(id)) {
            route.add(edges.get(edge));
        }
    }

    public Point2f getNextRoutePoint(int laneID) {
        // FIXME!!
        Lane lane = getNextLane(laneID);
        Point2f point = lane.getInnerPoints().get(nextPoint);


        // Increment the point pointers
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
        return point;
    }

    public int getNumberOfLanePoints(int lane) {
        Lane ln = getNextLane(lane);
        if (ln == null) {
            return 0;
        } else {
            return ln.getInnerPoints().size();
        }
    }

    private Lane getNextLane(int currentLane) {
        //FIXME!!!
        String getter = String.format("%s_%d", route.get(nextEdge).getId(),
                currentLane);
        return route.get(nextEdge).getLanes().get(getter);

    }

    public Point2f getInitialPosition() {
        return route.get(0).getLanes().values().iterator().next().getInnerPoints().get(0);
    }
}
