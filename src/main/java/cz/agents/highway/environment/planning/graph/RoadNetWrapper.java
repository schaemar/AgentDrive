package cz.agents.highway.environment.planning.graph;

import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.environment.roadnet.Network;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.WeightedGraph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.GraphDelegator;
import tt.euclid2d.Line;
import tt.euclid2d.Point;

import javax.vecmath.Point2f;
import javax.vecmath.Vector2d;
import java.util.*;

/**
 * Generates planning directed graph from the SUMO road network
 * Created by wmatex on 18.11.14.
 */
public class RoadNetWrapper extends GraphDelegator<Point, Line> implements DirectedGraph<Point, Line> {
    private static final int OVERTAKE_OFFSET = 3;
    private static long numVertex, numEdge = 0;

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
    public static RoadNetWrapper create(String startingLaneIdx) {
        Network network = Network.getInstance();
        HashSet<String> closedList = new HashSet<String>();
        DirectedGraph<Point, Line> graph = new DefaultDirectedWeightedGraph<Point, Line>(Line.class);

        Lane startingLane = network.getLanes().get(startingLaneIdx);
        // Traverse the lanes using BFS
        numEdge = numVertex = 0;
        RoadNetWrapper.traverse(startingLane, graph, closedList);

//        System.out.println("Vertex: "+numVertex+", edge: "+numEdge);
        return new RoadNetWrapper(graph);
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
                graph.addVertex(nextPoint);
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
        direction.scale(Lane.INNER_POINTS_STEP_SIZE);
        p = new Point(start);
        Point lastPoint = new Point(p);
        p.add(direction);
        graph.addVertex(new Point(start));
        while (p.distance(end) > Lane.INNER_POINTS_STEP_SIZE) {
            graph.addVertex(new Point(p));
            graph.addEdge(new Point(lastPoint), new Point(p), new Line(new Point(lastPoint), new Point(p)));
            lastPoint = new Point(p);
            p.add(direction);
        }
        graph.addVertex(end);
        graph.addEdge(lastPoint, end, new Line(lastPoint, end));
    }

    private static void traverse(Lane lane, DirectedGraph<Point, Line> graph, Set<String> visited) {
        visited.add(lane.getLaneId());
        Set<Point> closedVertexList = new HashSet<Point>();
        Queue<BFSPoint> openVertexList = new LinkedList<BFSPoint>();
        Point startPoint = innerPointToPoint(lane, 0);
        openVertexList.offer(new BFSPoint(startPoint, 0, lane));

        // BFS
        while (!openVertexList.isEmpty()) {
            BFSPoint current = openVertexList.poll();

            graph.addVertex(current.point);
            closedVertexList.add(current.point);

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
