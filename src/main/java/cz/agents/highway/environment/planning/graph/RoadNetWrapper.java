package cz.agents.highway.environment.planning.graph;

import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.environment.roadnet.Network;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.WeightedGraph;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.graph.DefaultDirectedWeightedGraph;
import org.jgrapht.graph.GraphDelegator;
import tt.euclid2i.Line;
import tt.euclid2i.Point;

import javax.vecmath.Point2f;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;

/**
 * Generates planning directed graph from the SUMO road network
 * Created by wmatex on 18.11.14.
 */
public class RoadNetWrapper extends GraphDelegator<Point, Line> implements DirectedGraph<Point, Line> {
    private static final int OVERTAKE_OFFSET = 3;
    private static long numVertex, numEdge = 0;
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
        // Traverse the lanes using DFS
        numEdge = numVertex = 0;
        RoadNetWrapper.traverse(startingLane, graph, closedList, null);

//        System.out.println("Vertex: "+numVertex+", edge: "+numEdge);
        return new RoadNetWrapper(graph);
    }

    private static void traverse(Lane lane, DirectedGraph<Point, Line> graph, Set<String> visited, Point lastPoint) {
        if (lane == null || visited.contains(lane.getLaneId())) {
            return;
        }
        visited.add(lane.getLaneId());

        // Add left and right lanes
        Lane left = lane.getLaneLeft(), right = lane.getLaneRight();
        traverse(left, graph, visited, lastPoint);
        traverse(right, graph, visited, lastPoint);

        ArrayList<Point2f> leftPoints = null, rightPoints = null;
        if (left != null) {
            leftPoints = left.getInnerPoints();
        }
        if (right != null) {
            rightPoints = right.getInnerPoints();
        }

        // Add vertexes and edges to graph
        int i = 0;
        for (Point2f innerPoint: lane.getInnerPoints()) {
            final Point p = new Point(Math.round(innerPoint.x), Math.round(innerPoint.y));
            graph.addVertex(p);
            if (lastPoint != null) {
                graph.addEdge(lastPoint, p, new Line(lastPoint, p));
                ++numVertex;
            }

            // Generate edges to left and right lanes for overtake
            ArrayList<Point2f> neighbourLanePoints = leftPoints;
            for (int j = 0; j < 2; j++) {
                if (neighbourLanePoints != null && i + OVERTAKE_OFFSET < neighbourLanePoints.size()) {
                    Point2f tmp = neighbourLanePoints.get(i + OVERTAKE_OFFSET);
                    final Point neighbourPoint = new Point(Math.round(tmp.x), Math.round(tmp.y));
                    if (graph.containsVertex(neighbourPoint)) {
                        Line edge = new Line(p, neighbourPoint);
                        graph.addEdge(p, neighbourPoint, edge);
                        ++numEdge;

                        // Check also reverse edge
                        if (i - OVERTAKE_OFFSET >= 0) {
                            tmp = neighbourLanePoints.get(i - OVERTAKE_OFFSET);
                            final Point reversePoint = new Point(Math.round(tmp.x), Math.round(tmp.y));
                            if (!graph.containsEdge(reversePoint, p)) {
                                Line reversed = new Line(reversePoint, p);
                                if (!graph.containsVertex(reversePoint)) {
                                    graph.addVertex(reversePoint);
                                }
                                graph.addEdge(reversePoint, p, reversed);
                                ++numEdge;
                            }
                        }
                    }
                }
                neighbourLanePoints = rightPoints;
            }

            lastPoint = p;
            i++;
        }

        // Traverse all outgoing lanes
        for (Lane outgoing: lane.getOutgoingLanes()) {
            if (!visited.contains(outgoing.getLaneId())) {
                traverse(outgoing, graph, visited, lastPoint);
            }
        }
    }

    @Override
    public double getEdgeWeight(Line edge) {
        return edge.getDistance();
    }
}
