package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.highway.environment.planning.graph.ControlEffortWrapper;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.graph.DefaultDirectedGraph;
import org.jgrapht.util.Goal;
import tt.discrete.Trajectory;
import tt.discrete.vis.TrajectoryLayer;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.trajectory.StraightSegmentTrajectory;
import tt.euclid2i.vis.ProjectionTo2d;
import tt.euclidtime3i.L2Heuristic;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.discretization.ConstantSpeedTimeExtension;
import tt.euclidtime3i.discretization.FreeOnTargetWaitExtension;
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.vis.TimeParameterProjectionTo2d;
import tt.vis.GraphLayer;
import tt.vis.TimeParameterHolder;

import javax.vecmath.Point2f;
import javax.vecmath.Point2i;
import javax.vecmath.Point3f;
import java.awt.Color;
import java.util.*;
import java.util.List;

/**
 * Planning agent implementation using the
 * Asynchronous Decentralized Prioritized Planning Algorithm, see: http://agents.fel.cvut.cz/~cap/research/adpp/
 *
 * Created by wmatex on 13.10.14.
 */
public class ADPPAgent extends Agent {
    private static final int OVERTAKE_OFFSET = 5;
    private static final int SKIP_POINTS = 3;
    private static final int MOVE_PENALTY = 1;
    private static final int RADIUS = 3;
    private static final int SPEED = 1;
    private static final int MAX_TIME = 1000;
    private static final Color[] TRAJ_COLORS = { Color.GREEN, Color.MAGENTA };
    private static int INDEX = 0;
    private static ArrayList<Region> agentTrajectories = new ArrayList<Region>();

    DirectedGraph<Point, Line> planningGraph;

    tt.euclid2i.Trajectory trajectory;

    public ADPPAgent(int id) {
        super(id);

        planningGraph = buildPlanningGraph();

        Edge lastEdge = navigator.getRoute().get(navigator.getRoute().size()-1);
        List<Point2f> innerPs = lastEdge.getLanes().values().iterator().next().getInnerPoints();
        Point2f lastPoint = innerPs.get(innerPs.size()-1);
        final Point goal = new Point(Math.round(lastPoint.x), Math.round(lastPoint.y));

        if (INDEX == 1) {
            navigator.changeLaneLeft();
            Lane l = lastEdge.getLanes().values().iterator().next();
            innerPs = l.getLaneRight().getInnerPoints();
            lastPoint = innerPs.get(innerPs.size()-1);
            goal.set(Math.round(lastPoint.x), Math.round(lastPoint.y));
        }
        Point start = new Point(Math.round(navigator.getRoutePoint().x), Math.round(navigator.getRoutePoint().y));


        System.out.println("Agent "+INDEX+" planning");
        System.out.println("Start: "+start);
        System.out.println("Goal: "+goal);
        DirectedGraph<tt.euclidtime3i.Point, Straight> graph = new ConstantSpeedTimeExtension(planningGraph, MAX_TIME, new int[] {SPEED}, agentTrajectories, 1, 1);
        graph = new FreeOnTargetWaitExtension(graph, goal);
        graph = new ControlEffortWrapper(graph);
        final GraphPath<tt.euclidtime3i.Point, Straight> path = AStarShortestPathSimple.findPathBetween(graph, new L2Heuristic(goal),
                new tt.euclidtime3i.Point(start.x, start.y, 0), new Goal<tt.euclidtime3i.Point>() {
            @Override
            public boolean isGoal(tt.euclidtime3i.Point point) {
                return (goal.distance(point.getPosition()) < 1);
            }
        });
        if (path == null) {
            System.out.println("No path found!");
            return;
        }
        System.out.println("Path end time: "+path.getEndVertex().getTime());
        trajectory = new StraightSegmentTrajectory(path, path.getEndVertex().getTime());
        agentTrajectories.add(new MovingCircle(trajectory, RADIUS));

        VisManager.registerLayer(TrajectoryLayer.create(new TrajectoryLayer.TrajectoryProvider<Point>() {
            @Override
            public Trajectory<Point> getTrajectory() {
                return trajectory;
            }
        }, new ProjectionTo2d(), TRAJ_COLORS[INDEX++], 1, MAX_TIME, 'g'));
        VisManager.registerLayer(tt.euclidtime3i.vis.RegionsLayer.create(new tt.euclidtime3i.vis.RegionsLayer.RegionsProvider() {

            @Override
            public Collection<tt.euclidtime3i.Region> getRegions() {
                return agentTrajectories;
            }
        }, new TimeParameterProjectionTo2d(TimeParameterHolder.time), Color.BLACK, Color.BLACK));
    }

    /**
     * Build the planning graph based on the road network
     */
    private DirectedGraph<Point, Line> buildPlanningGraph() {
        Network network = Network.getInstance();
        HashSet<String> closedList = new HashSet<String>();
        DirectedGraph<Point, Line> graph = new DefaultDirectedGraph<Point, Line>(Line.class);

        Lane startingLane = network.getLanes().get(navigator.getUniqueLaneIndex());
        // Traverse the lanes using DFS
        traverse(startingLane, graph, closedList, null);

        VisLayer graphLayer;

        if (INDEX == 0) {
            graphLayer = GraphLayer.create(new GraphLayer.GraphProvider<Point, Line>() {
                @Override
                public Graph<Point, Line> getGraph() {
                    return planningGraph;
                }
            }, new ProjectionTo2d(), Color.BLUE, Color.RED, 2, 3);
            VisManager.registerLayer(graphLayer);
        }

        return graph;
    }

    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        this.sensor.registerReaction(new Reaction() {
            @Override
            public void react(Event event) {
                if (event.getType().equals(HighwayEventType.UPDATED)) {
                    actuator.act(agentReact());
                }
            }
        });
    }

    private List<Action> agentReact() {
        LinkedList<Action> actions = new LinkedList<Action>(); // list of actions for the simulator
        RoadObject me = sensor.senseCurrentState(); // my current state
        // Simulator did not send update yet
        if (me == null) {
            actions.add(new WPAction(id, 0d, getInitialPosition(), 0));
            return actions;
        }

        int time = (int) Math.floor(me.getUpdateTime()/1000);

        Point p = trajectory.get(time);
        if (p == null) {
            p=trajectory.get(trajectory.getMaxTime());
        }
        Point3f planP = new Point3f(p.x, p.y, 0);
        actions.add(new WPAction(me.getId(), me.getUpdateTime(), planP, SPEED));
        return actions;
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
