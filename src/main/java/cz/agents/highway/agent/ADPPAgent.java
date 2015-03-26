package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.ScreenTextLayer;
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
import tt.euclid2d.util.Intersection;
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
import java.io.IOException;
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
    private static final int RADIUS = 2;
    private static final int SPEED = 1;
    private static final float[] SPEEDS = new float[] {1};
    private static final double WAIT_PENALTY = 0;
    private static final double MOVE_PENALTY = 0;
    private static final int WAIT_DURATION = 1;
    private static final int MAX_TIME = 2000;
    private static final int MAX_PRIORITY = 1000;
    private static final Timer globalTimer = new Timer(true);
    private static final TimeParameter timeParameter = new TimeParameter();
    private static ArrayList<Region> agentTrajectories = new ArrayList<Region>();
    private static float maxTime = -1;
    private static int nOfReplans = 0;

    DirectedGraph<Point, Line> spatialGraph;
    DirectedGraph<tt.euclidtime3i.Point, Straight> planningGraph;

    private PriorityRelation priorityRelation;

    Timer timer = new Timer(false);

    tt.euclid2i.Trajectory trajectory;
    VisLayer trajectoryLayer = null;
    VisLayer movingRegionLayer = null;
    Point start, goal;
    long expanded = 0;
    static long notFound = 0;
    String heuristic;
    float[] speeds;
    double waitPenalty;
    double movePenalty;
    int waitDuration;
    int lastTime = 0;

    int speed = 0;

    boolean needsReplan = true;

    private Map<Integer, Region> movingObstaclesSaved = new LinkedHashMap<Integer, Region>();

    // Should we print additional information?
    boolean verbose = false;

    public ADPPAgent(int id) {
        this(id, SPEEDS, WAIT_PENALTY, MOVE_PENALTY, "perfect", WAIT_DURATION, true, true, false);
    }

    public ADPPAgent(int id, float[] speeds, double waitPenalty, double movePenalty,
                     String heuristic, int waitDuration, boolean vis, boolean verbose, boolean emptyTrajectories) {
        super(id);
        this.verbose = verbose;
        this.heuristic = heuristic;
        this.speeds = speeds;
        this.waitPenalty = waitPenalty;
        this.movePenalty = movePenalty;
        this.waitDuration = waitDuration;

//        this.speed = (int) (Math.random() * 3 + 1);
        this.speed = 1;
        System.out.println("Agent " + id + ", speed = "+speed);

        // Empty agent trajectories
        if (emptyTrajectories) {
            agentTrajectories.clear();
        }

        timer.reset();
        spatialGraph = RoadNetWrapper.create(navigator.getUniqueLaneIndex());
        VisLayer graphLayer;

        if (id == 1 && vis) {
            VisManager.registerLayer(ParameterControlLayer.create(timeParameter));
            graphLayer = KeyToggleLayer.create("g", true, GraphLayer.create(new GraphLayer.GraphProvider<Point, Line>() {
                @Override
                public Graph<Point, Line> getGraph() {
                    return spatialGraph;
                }
            }, new ProjectionTo2d(), Color.BLUE, Color.RED, 2, 3));
            VisManager.registerLayer(graphLayer);
        }
    }


    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        priorityRelation = new TrafficRulesPriorityRelation(sensor);
        this.sensor.registerReaction(new Reaction() {
            @Override
            public void react(Event event) {
                if (event.getType().equals(HighwayEventType.UPDATED)) {
                    actuator.act(agentReact());
                } else if (event.getType().equals(HighwayEventType.TRAJECTORY_CHANGED)) {
                    int key = (Integer) event.getContent();
                    if (priorityRelation.higherPriority().contains(key)) {
                        needsReplan = true;
                    }
                }
            }
        });
    }

    public long getExpandedNodes() {
        return expanded;
    }

    private List<Action> agentReact() {
        LinkedList<Action> actions = new LinkedList<Action>(); // list of actions for the simulator
        RoadObject me = sensor.senseCurrentState(); // my current state
        // Simulator did not send update yet
        int time = (int) Math.floor(me.getUpdateTime() / 1000);
        if (me == null) {
            actions.add(new WPAction(id, 0d, getInitialPosition(), 0));
        } else {
            if (needsReplan) {
                Map<Integer, Region> trajectories = sensor.senseTrajectories();
                List<Integer> higherPriority = priorityRelation.higherPriority();
                for (int id: higherPriority) {
                    Region t = trajectories.get(id);
                    if (t != null) {
                        movingObstaclesSaved.put(id, t);
                    }
                }
                System.out.println("AgentID: " + id);
//                planningGraph = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, new int[] {speed}, new ArrayList<Region>(agentTrajectories), 1, 1);
                planningGraph = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, new int[]{speed}, movingObstaclesSaved.values(), waitDuration, 1);
//                planningGraph = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, SPEEDS, new ArrayList<Region>(), 1, 1);
                //planningGraph = new FreeOnTargetWaitExtension(planningGraph, goal);
                planningGraph = new ControlEffortWrapper(planningGraph, movePenalty, waitPenalty);
                // Do the planning
                this.plan(heuristic);
                needsReplan = false;
                if (trajectory != null) {
                    lastTime = time;
                    if (movingRegionLayer != null) {
                        VisManager.unregisterLayer(movingRegionLayer);
                    }
                    movingRegionLayer = KeyToggleLayer.create("" + id, true, tt.euclidtime3i.vis.RegionsLayer.create(new tt.euclidtime3i.vis.RegionsLayer.RegionsProvider() {

                        @Override
                        public Collection<Region> getRegions() {
                            List<Region> regions = new ArrayList<Region>(1);
                            regions.add(new MovingCircle(trajectory, RADIUS));
                            return regions;
                        }
                    }, new TimeParameterProjectionTo2d(timeParameter), AgentColors.getColorForAgent(id), AgentColors.getColorForAgent(id)));
                    VisManager.registerLayer(movingRegionLayer);
                }
            }

            if (trajectory == null) {
                actions.add(new WPAction(id, 0d, getInitialPosition(), 0));
            } else {
                Point p = trajectory.get(time);
                if (p == null) {
                    //needsReplan = true;
                    p = trajectory.get(trajectory.getMaxTime());
                }
                Point3f planP = new Point3f(p.x, p.y, 0);
                actions.add(new WPAction(me.getId(), me.getUpdateTime(), planP, speed));
            }
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
        if (trajectoryLayer != null) {
            VisManager.unregisterLayer(trajectoryLayer);
        }
        Edge lastEdge = navigator.getRoute().get(navigator.getRoute().size()-1);
        Iterator<Lane> laneIterator = lastEdge.getLanes().values().iterator();
        Lane l = laneIterator.next();
        List<Point2f> innerPs = l.getInnerPoints();
        Point2f lastPoint = innerPs.get(innerPs.size()-1);
        Point3f current = sensor.senseCurrentState().getPosition();
        Point2f routePoint = navigator.getRoutePoint();
        start = new Point(Math.round(routePoint.x), Math.round(routePoint.y));
        goal = new Point(Math.round(lastPoint.x), Math.round(lastPoint.y));
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
                    //return (point.getTime() >= MAX_TIME || goal.distance(point.getPosition()) < 1);
                    return (goal.distance(point.getPosition()) < 1);
                }
            });
        float planTime = timer.getRawElapsedTime();
        if (planTime > maxTime) {
            maxTime = planTime;
        }
        nOfReplans++;
        print("Planning took: " + timer.getElapsedTime() + ", expanded nodes: " + expanded);
        print("Maximal time: "+maxTime+", number of replans: "+nOfReplans);

        if (path == null) {
            trajectory = null;
            ++notFound;
            print("No path found! Sum: "+notFound);
            return;
        }
        print("Trajectory end time: "+path.getEndVertex().getTime());
        trajectory = new StraightSegmentTrajectory(path, path.getEndVertex().getTime());
        MovingCircle region = new MovingCircle(trajectory, RADIUS);

        // Dispatch an event
        actuator.getEventProcessor().addEvent(HighwayEventType.TRAJECTORY_UPDATED, null, null,
                new AbstractMap.SimpleEntry<Integer, Region>(this.id, region));
/*        trajectoryLayer = TrajectoryLayer.create(new TrajectoryLayer.TrajectoryProvider<Point>() {
            @Override
            public Trajectory<Point> getTrajectory() {
                return trajectory;
            }
        }, new ProjectionTo2d(), AgentColors.getColorForAgent(id), 1, trajectory.getMaxTime(), 't');
        VisManager.registerLayer(trajectoryLayer);*/
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
