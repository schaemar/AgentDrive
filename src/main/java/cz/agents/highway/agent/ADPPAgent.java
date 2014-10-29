package cz.agents.highway.agent;

import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.environment.roadnet.Network;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.graph.DefaultDirectedGraph;
import tt.discrete.Trajectory;
import tt.discrete.vis.TrajectoryLayer;
import tt.euclid2i.HeuristicToPoint;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.vis.ProjectionTo2d;
import tt.vis.GraphLayer;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import java.awt.*;
import java.util.*;
import java.util.List;

/**
 * Planning agent implementation using the
 * Asynchronous Decentralized Prioritized Planning Algorithm, see: http://agents.fel.cvut.cz/~cap/research/adpp/
 *
 * Created by wmatex on 13.10.14.
 */
public class ADPPAgent extends Agent {
    private static final int OVERTAKE_OFFSET = 20;
    private static final int SKIP_POINTS = 3;

    DirectedGraph<Point, Line> planningGraph;

    final RouteNavigator routeNavigator;

    public ADPPAgent(int id) {
        super(id);
        routeNavigator = new RouteNavigator(id);

        planningGraph = buildPlanningGraph();

        Edge lastEdge = routeNavigator.getRoute().get(routeNavigator.getRoute().size()-1);
        List<Point2f> innerPs = lastEdge.getLanes().values().iterator().next().getInnerPoints();
        Point2f lastPoint = innerPs.get(innerPs.size()-1);
        Point goal = new Point(Math.round(lastPoint.x), Math.round(lastPoint.y));
        Point start = new Point(Math.round(routeNavigator.getRoutePoint().x), Math.round(routeNavigator.getRoutePoint().y));

        final GraphPath<Point, Line> path = AStarShortestPathSimple.findPathBetween(planningGraph, new HeuristicToPoint(goal), start, goal);
        System.out.println(path.getStartVertex());
        System.out.println(path.getEndVertex());

        VisManager.registerLayer(TrajectoryLayer.create(new TrajectoryLayer.TrajectoryProvider<Point>() {
            @Override
            public Trajectory<Point> getTrajectory() {
                return new Trajectory<Point>() {
                    @Override
                    public int getMinTime() {
                        return 0;
                    }

                    @Override
                    public int getMaxTime() {
                        return path.getEdgeList().size()-1;
                    }

                    @Override
                    public Point get(int i) {
                        return path.getEdgeList().get(i).getStart();
                    }
                };
            }
        }, new ProjectionTo2d(), Color.GREEN, 1, 100001, 'g'));
    }

    /**
     * Build the planning graph based on the road network
     */
    private DirectedGraph<Point, Line> buildPlanningGraph() {
        Network network = Network.getInstance();
        HashSet<String> closedList = new HashSet<String>();
        DirectedGraph<Point, Line> graph = new DefaultDirectedGraph<Point, Line>(Line.class);

        Lane startingLane = network.getLanes().get(routeNavigator.getUniqueLaneIndex());
        // Traverse the lanes using DFS
        traverse(startingLane, graph, closedList, null);

        planningGraph =

        VisLayer graphLayer;

        graphLayer = GraphLayer.create(new GraphLayer.GraphProvider<Point, Line>() {
            @Override
            public Graph<Point, Line> getGraph() {
                return planningGraph;
            }
        }, new ProjectionTo2d(), Color.BLUE, Color.RED, 2, 3);
        VisManager.registerLayer(graphLayer);

        return graph;
    }

    private void traverse(Lane lane, DirectedGraph<Point, Line> graph, Set<String> visited, Point lastPoint) {
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
            }

            // Generate edges to left and right lanes for overtake
            ArrayList<Point2f> neighbourLanePoints = leftPoints;
            for (int j = 0; j < 2; j++) {
                if (neighbourLanePoints != null && i + OVERTAKE_OFFSET < neighbourLanePoints.size()) {
                    Point2f tmp = neighbourLanePoints.get(i + OVERTAKE_OFFSET);
                    final Point neighbourPoint = new Point(Math.round(tmp.x), Math.round(tmp.y));
                    if (graph.containsVertex(neighbourPoint)) {
                        graph.addEdge(p, neighbourPoint, new Line(p, neighbourPoint));

                        // Check also reverse edge
                        if (i - OVERTAKE_OFFSET >= 0) {
                            tmp = neighbourLanePoints.get(i - OVERTAKE_OFFSET);
                            final Point reversePoint = new Point(Math.round(tmp.x), Math.round(tmp.y));
                            if (!graph.containsEdge(reversePoint, p)) {
                                graph.addEdge(reversePoint, p, new Line(reversePoint, p));
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
}
