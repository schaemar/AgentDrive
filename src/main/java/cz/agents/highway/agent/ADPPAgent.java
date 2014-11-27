package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.highway.environment.planning.Timer;
import cz.agents.highway.environment.planning.graph.ControlEffortWrapper;
import cz.agents.highway.environment.planning.graph.RoadNetWrapper;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;
import cz.agents.highway.vis.AgentColors;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.GraphPath;
import org.jgrapht.alg.AStarShortestPathSimple;
import org.jgrapht.graph.EdgeReversedGraph;
import org.jgrapht.util.Goal;
import org.jgrapht.util.HeuristicToGoal;
import tt.discrete.Trajectory;
import tt.discrete.vis.TrajectoryLayer;
import tt.euclid2i.Line;
import tt.euclid2i.Point;
import tt.euclid2i.trajectory.StraightSegmentTrajectory;
import tt.euclid2i.vis.ProjectionTo2d;
import tt.euclidtime3i.L2Heuristic;
import tt.euclidtime3i.Region;
import tt.euclidtime3i.ShortestPathHeuristic;
import tt.euclidtime3i.discretization.ConstantSpeedTimeExtension;
import tt.euclidtime3i.discretization.FreeOnTargetWaitExtension;
import tt.euclidtime3i.discretization.Straight;
import tt.euclidtime3i.region.MovingCircle;
import tt.euclidtime3i.vis.TimeParameter;
import tt.euclidtime3i.vis.TimeParameterProjectionTo2d;
import tt.vis.GraphLayer;
import tt.vis.ParameterControlLayer;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import java.awt.*;
import java.util.*;
import java.util.List;

/**
 * Planning agent implementation using the
 * Asynchronous Decentralized Prioritized Planning Algorithm
 * see: http://agents.fel.cvut.cz/~cap/research/adpp/
 *
 * Created by wmatex on 13.10.14.
 */
public class ADPPAgent extends Agent {
    private static final int RADIUS = 5;
    private static final int SPEED = 1;
    private static final float[] SPEEDS = new float[] {1};
    private static final double WAIT_PENALTY = 100;
    private static final double MOVE_PENALTY = 1;
    private static final int WAIT_DURATION = 1;
    private static final int MAX_TIME = 10000;
    private static final Timer globalTimer = new Timer(true);
    private static final TimeParameter timeParameter = new TimeParameter();
    private static ArrayList<Region> agentTrajectories = new ArrayList<Region>();

    DirectedGraph<Point, Line> spatialGraph;
    DirectedGraph<tt.euclidtime3i.Point, Straight> planningGraph;

    Timer timer = new Timer(false);

    tt.euclid2i.Trajectory trajectory;
    VisLayer trajectoryLayer = null;
    Point start, goal;
    long expanded = 0;

    // Should we print additional information?
    boolean verbose = true;

    public ADPPAgent(int id) {
        this(id, SPEEDS, WAIT_PENALTY, MOVE_PENALTY, "perfect", WAIT_DURATION, true, true, false);
    }

    public ADPPAgent(int id, float[] speeds, double waitPenalty, double movePenalty,
                     String heuristic, int waitDuration, boolean vis, boolean verbose, boolean emptyTrajectories) {
        super(id);
        this.verbose = verbose;

        // Empty agent trajectories
        if (emptyTrajectories) {
            agentTrajectories.clear();
        }

        timer.reset();
        spatialGraph = RoadNetWrapper.create(navigator.getUniqueLaneIndex());
        VisLayer graphLayer;

        if (id == 0 && vis) {
            VisManager.registerLayer(ParameterControlLayer.create(timeParameter));
            graphLayer = KeyToggleLayer.create("g", true, GraphLayer.create(new GraphLayer.GraphProvider<Point, Line>() {
                @Override
                public Graph<Point, Line> getGraph() {
                    return spatialGraph;
                }
            }, new ProjectionTo2d(), Color.BLUE, Color.RED, 2, 3));
            VisManager.registerLayer(graphLayer);
        }
        print("Building graph: "+timer.getElapsedTime());

        Edge lastEdge = navigator.getRoute().get(navigator.getRoute().size()-1);
        Iterator<Lane> laneIterator = lastEdge.getLanes().values().iterator();
        for (int i = 0; i < id; i++) {
            navigator.changeLaneLeft();
            laneIterator.next();
        }
        Lane l = laneIterator.next();
        List<Point2f> innerPs = l.getInnerPoints();
        Point2f lastPoint = innerPs.get(innerPs.size()-1);
        goal = new Point(Math.round(lastPoint.x), Math.round(lastPoint.y));
        start = new Point(Math.round(navigator.getRoutePoint().x), Math.round(navigator.getRoutePoint().y));

        print("Agent "+id+" planning");
//        planningGraph = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, new int[] {speed}, new ArrayList<Region>(agentTrajectories), 1, 1);
        planningGraph = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, speeds, new ArrayList<Region>(agentTrajectories), 0, 1);
//        planningGraph = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, SPEEDS, new ArrayList<Region>(), 1, 1);
        planningGraph = new FreeOnTargetWaitExtension(planningGraph, goal);
        planningGraph = new ControlEffortWrapper(planningGraph, movePenalty, waitPenalty);
        // Do the planning
        this.plan(heuristic);
        VisManager.registerLayer(KeyToggleLayer.create(""+id, true, tt.euclidtime3i.vis.RegionsLayer.create(new tt.euclidtime3i.vis.RegionsLayer.RegionsProvider() {

            @Override
            public Collection<Region> getRegions() {
                List<Region> regions = new ArrayList<Region>(1);
                regions.add(new MovingCircle(trajectory, RADIUS));
                return regions;
            }
        }, new TimeParameterProjectionTo2d(timeParameter), AgentColors.getColorForAgent(id), AgentColors.getColorForAgent(id))));
        print("Sum time: "+globalTimer.getElapsedTime());
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
        int time = (int) Math.floor(me.getUpdateTime() / 1000);
        if (me == null || trajectory == null) {
            actions.add(new WPAction(id, 0d, getInitialPosition(), 0));
        } else {

            Point p = trajectory.get(time);
            if (p == null) {
                p = trajectory.get(trajectory.getMaxTime());
            }
            Point3f planP = new Point3f(p.x, p.y, 0);
            actions.add(new WPAction(me.getId(), me.getUpdateTime(), planP, SPEED));
        }
        // Replan every 10 seconds
        if (time > 0 && time % 10 == 0) {
//            print("Agent " + id + " replanning...");
//            this.plan();
        }
        return actions;
    }


    /**
     * Plan the optimal non-collision trajectory
     */
    private void plan(String heuristic) {
        print("Start: "+start);
        print("Goal: " + goal);
        expanded = 0;

        HeuristicToGoal<tt.euclidtime3i.Point> heuristicToGoal;
        if (heuristic.equals("distance")) {
            heuristicToGoal = new L2Heuristic(goal);
        } else {
            timer.reset();
            DirectedGraph<Point, Line> reversed = new EdgeReversedGraph<Point, Line>(spatialGraph);
            print("Reversing graph: " + timer.getElapsedTime());
            timer.reset();
            heuristicToGoal = new ShortestPathHeuristic(reversed, goal);
            print("Creating heuristic: " + timer.getElapsedTime());
        }
        timer.reset();
        GraphPath<tt.euclidtime3i.Point, Straight> path = AStarShortestPathSimple.findPathBetween(planningGraph,
            //new SpaceTimeHeuristic(goal, start),
//            new L2Heuristic(goal),
              heuristicToGoal,
            new tt.euclidtime3i.Point(start.x, start.y, 0), new Goal<tt.euclidtime3i.Point>() {
                @Override
                public boolean isGoal(tt.euclidtime3i.Point point) {
                    ++expanded;
//                    print("Trying: "+point);
                    return (goal.distance(point.getPosition()) < 1);
                }
            });
        print("Planning took: " + timer.getElapsedTime()+", expanded nodes: "+expanded);
        if (path == null) {
            trajectory = null;
            print("No path found!");
            return;
        }
        trajectory = new StraightSegmentTrajectory(path, path.getEndVertex().getTime());
        MovingCircle region = new MovingCircle(trajectory, RADIUS);
        try {
            agentTrajectories.set(id, region);
        } catch (IndexOutOfBoundsException e) {
            agentTrajectories.add(region);
        }
        trajectoryLayer = TrajectoryLayer.create(new TrajectoryLayer.TrajectoryProvider<Point>() {
            @Override
            public Trajectory<Point> getTrajectory() {
                return trajectory;
            }
        }, new ProjectionTo2d(), AgentColors.getColorForAgent(id), 1, trajectory.getMaxTime(), 't');
        VisManager.registerLayer(trajectoryLayer);
    }

    /**
     * Print if verbose is set
     */
    private void print(String s) {
        if (verbose) {
            System.out.println(s);
        }
    }
}
