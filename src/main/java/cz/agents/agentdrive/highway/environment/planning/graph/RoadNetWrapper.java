package cz.agents.agentdrive.highway.environment.planning.graph;

import cz.agents.agentdrive.highway.environment.roadnet.Junction;
import cz.agents.agentdrive.highway.environment.roadnet.Lane;
import cz.agents.agentdrive.highway.environment.roadnet.LaneImpl;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.GraphDelegator;
import tt.euclid2d.Line;
import tt.euclid2d.Point;

import javax.vecmath.Point2d;
import javax.vecmath.Point2f;
import javax.vecmath.Vector2d;
import java.util.*;

/**
 * Generates planning directed graph from the SUMO road network
 * Created by wmatex on 18.11.14.
 */
public class RoadNetWrapper extends GraphDelegator<Point, Line> implements DirectedGraph<Point, Line> {
    private static final int OVERTAKE_OFFSET = 5;
    private static final double JUNCTION_RADIUS = 7;
    private static long numVertex, numEdge = 0;
    private static Set<Point> junctionPoints = new HashSet<Point>();
    private static RoadNetwork network;

    private static class BFSPoint {
        public Point point;
        public int laneIndex;
        public Lane lane;

        private BFSPoint(Point point, int laneIndex, Lane lane) {
            this.point = point;
            this.laneIndex = laneIndex;
            this.lane = lane;
        }
    }


    public RoadNetWrapper(Graph<Point, Line> pointLineGraph) {
        super(pointLineGraph);
    }

    /**
     * Build the planning graph based on the road network
     */
    public static RoadNetWrapper create(RoadNetwork network, Lane startingLaneIdx) {
        RoadNetWrapper.network = network;
        HashSet<String> closedList = new HashSet<String>();
        DirectedGraph<Point, Line> graph = new DefaultDirectedWeightedGraph<Point, Line>(Line.class);

        LaneImpl startingLane = network.getLanes().get(startingLaneIdx);
        // Traverse the lanes using BFS
        numEdge = numVertex = 0;
        RoadNetWrapper.traverse(startingLane, graph, closedList);

//        System.out.println("Vertex: "+numVertex+", edge: "+numEdge);
        return new RoadNetWrapper(graph);
    }

    public boolean isInJunction(Point p) {
        return junctionPoints.contains(p);
    }

    private static Point innerPointToPoint(Lane lane, int index) {
        if (index < lane.getInnerPoints().size()) {
            Point2f innerPoint = lane.getInnerPoints().get(index);
            return new Point(innerPoint.x, innerPoint.y);
        } else {
            return null;
        }
    }

    private static void addEdge(DirectedGraph<Point, Line> graph, Queue<BFSPoint> openList, Set<Point> closedList, BFSPoint current, Lane lane, int index) {
        Point nextPoint = innerPointToPoint(lane, index);
        if (nextPoint != null) {
            if (!closedList.contains(nextPoint)) {
                openList.offer(new BFSPoint(nextPoint, index, lane));
                closedList.add(nextPoint);
                addGraphVertex(nextPoint, graph);
            }
            interpolate(graph, current.point, nextPoint);
//            graph.addEdge(current.point, nextPoint, new Line(current.point, nextPoint));
        }
    }

    private static void interpolate(DirectedGraph<Point, Line> graph, Point start, Point end) {
        Point p = new Point(end);
        p.sub(start);
        Vector2d direction = new Vector2d(p);
        direction.normalize();
        direction.scale(LaneImpl.INNER_POINTS_STEP_SIZE);
        p = new Point(start);
        Point lastPoint = new Point(p);
        p.add(direction);
        addGraphVertex(new Point(start), graph);
        while (p.distance(end) > LaneImpl.INNER_POINTS_STEP_SIZE) {
            addGraphVertex(new Point(p), graph);
            graph.addEdge(new Point(lastPoint), new Point(p), new Line(new Point(lastPoint), new Point(p)));
            lastPoint = new Point(p);
            p.add(direction);
        }
        addGraphVertex(end, graph);
        graph.addEdge(lastPoint, end, new Line(lastPoint, end));
    }

    private static void addGraphVertex(Point vertex, DirectedGraph<Point, Line> graph) {
        graph.addVertex(vertex);
        for (Junction junction: network.getJunctions().values()) {
            double radius = junctionRadius(junction);
            Point2d junctionCenter = new Point2d(junction.getCenter().x, junction.getCenter().y);
            if (vertex.distance(junctionCenter) < radius) {
                junctionPoints.add(vertex);
            }
        }
    }

    private static double junctionRadius(Junction junction) {
        return JUNCTION_RADIUS;
//        double max = -1;
//        for (Point2f p: junction.getShape()) {
//            double dist = p.distance(junction.getCenter());
//            max = Math.max(max, dist);
//        }
//        return max;
    }

    private static void traverse(LaneImpl lane, DirectedGraph<Point, Line> graph, Set<String> visited) {
        visited.add(lane.getLaneId());
        Set<Point> closedVertexList = new HashSet<Point>();
        Queue<BFSPoint> openVertexList = new LinkedList<BFSPoint>();
        Point startPoint = innerPointToPoint(lane, 0);
        openVertexList.offer(new BFSPoint(startPoint, 0, lane));
        closedVertexList.add(startPoint);

        // BFS
        while (!openVertexList.isEmpty()) {
            BFSPoint current = openVertexList.poll();

            addGraphVertex(current.point, graph);
//            closedVertexList.add(current.point);

            // Forward
            addEdge(graph, openVertexList, closedVertexList, current, current.lane, current.laneIndex+1);
            // Overtake
            Lane overtakeLane = current.lane.getLaneLeft();
            for (int i = 0; i < 2; ++i) {
                if (overtakeLane != null) {
                    addEdge(graph, openVertexList, closedVertexList, current, overtakeLane, current.laneIndex+OVERTAKE_OFFSET);
                }
                overtakeLane = current.lane.getLaneRight();
            }

            // Connections
            // We're at the end of the lane
            if (current.laneIndex >= current.lane.getInnerPoints().size() - 1) {
                for (Lane outgoing: current.lane.getOutgoingLanes()) {
                    if (!visited.contains(outgoing.getLaneId())) {
                        visited.add(outgoing.getLaneId());
//                        openVertexList.add(new BFSPoint(innerPointToPoint(outgoing, 0), 0, outgoing));
                        for (int i = 0; i < outgoing.getInnerPoints().size(); ++i) {
                            Point firstPoint = innerPointToPoint(outgoing, i);
                            if (firstPoint != null && !closedVertexList.contains(firstPoint)) {
                                addEdge(graph, openVertexList, closedVertexList, current, outgoing, i);
                                break;
                            }
                        }
                    }
                }
            }

        }

    }

    @Override
    public double getEdgeWeight(Line edge) {
        return edge.getDistance();
    }
}
