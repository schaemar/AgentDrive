package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.highway.agent.adpp.PriorityRelation;
import cz.agents.highway.agent.adpp.TrafficRulesPriorityRelation;
import cz.agents.highway.environment.planning.Timer;
import cz.agents.highway.environment.planning.dynamics.AccelerationDynamicConstraint;
import cz.agents.highway.environment.planning.dynamics.DynamicConstraint;
import cz.agents.highway.environment.planning.euclid2d.BasicSegmentedTrajectory;
import cz.agents.highway.environment.planning.euclid2d.DiscreteTrajectoryWrapper;
import cz.agents.highway.environment.planning.euclid2d.StraightSegmentTrajectory;
import cz.agents.highway.environment.planning.euclid2d.Trajectory;
import cz.agents.highway.environment.planning.euclid3d.SpeedPoint;
import cz.agents.highway.environment.planning.euclid4d.Point4d;
import cz.agents.highway.environment.planning.euclid4d.Region;
import cz.agents.highway.environment.planning.euclid4d.Straight;
import cz.agents.highway.environment.planning.euclid4d.region.MovingCircle;
import cz.agents.highway.environment.planning.graph.*;
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
import tt.euclid2d.Line;
import tt.euclid2d.Point;
import tt.euclid2d.vis.ProjectionTo2d;
import tt.euclidtime3i.vis.TimeParameter;
import tt.euclidtime3i.vis.TimeParameterProjectionTo2d;
import tt.vis.GraphLayer;
import tt.vis.ParameterControlLayer;

import javax.vecmath.Point2d;
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
    private static final int RADIUS = 2;
    private static final float[] SPEEDS = new float[] {1};
    private static final double WAIT_PENALTY = 0.1;
    private static final double MOVE_PENALTY = 0.01;
    private static final double WAIT_DURATION = 1d;
    private static final double MAX_ACC = 2d;
    private static final double MAX_SPEED = 5d;
    private static final double REPLAN_THRESHOLD = 3d;
    private static final int MAX_TIME = 10;
    private static final Timer globalTimer = new Timer(true);
    private static final TimeParameter timeParameter = new TimeParameter();
    private static float maxTime = -1;
    private static int nOfReplans = 0;

    DirectedGraph<Point, Line> spatialGraph;
    DirectedGraph<Point4d, Straight> planningGraph;

    private PriorityRelation priorityRelation;
    private Set<Integer> finalPlans = new HashSet<Integer>();

    Timer timer = new Timer(false);

    Trajectory trajectory;
    VisLayer trajectoryLayer = null;
    VisLayer movingRegionLayer = null;
    Point goal;
    long expanded = 0;
    static long notFound = 0;
    String heuristic;
    float[] speeds;
    double waitPenalty;
    double movePenalty;
    double waitDuration;
    double time = 0;
    boolean planSent = false;

    DynamicConstraint dynamicConstraint = new AccelerationDynamicConstraint(MAX_ACC, MAX_SPEED);

    boolean needsReplan = true;

    // Should we print additional information?
    boolean verbose = false;

    public ADPPAgent(int id) {
        this(id, SPEEDS, WAIT_PENALTY, MOVE_PENALTY, "perfect", WAIT_DURATION, true, true, false);
    }

    public ADPPAgent(int id, float[] speeds, double waitPenalty, double movePenalty,
                     String heuristic, double waitDuration, boolean vis, boolean verbose, boolean emptyTrajectories) {
        super(id);
        this.verbose = verbose;
        this.heuristic = heuristic;
        this.speeds = speeds;
        this.waitPenalty = waitPenalty;
        this.movePenalty = movePenalty;
        this.waitDuration = waitDuration;

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
                    finalPlans.clear();
                    planSent = false;
                    RoadObject me = sensor.senseCurrentState();
                    time = me.getUpdateTime() / 1000;
                    if (shouldReplan()) {
                        replan();
                    } else if (trajectory != null && !replanTime()) {
                        actuator.act(agentReact());
                    }
                } else if (event.getType().equals(HighwayEventType.TRAJECTORY_CHANGED)) {
                    int key = (Integer) event.getContent();
                    if (key == id && !planSent) {
                        actuator.act(agentReact());
                    }
                } else if (event.getType().equals(HighwayEventType.NEW_PLAN)) {
                    List<Action> actions = (List<Action>) event.getContent();
                    int carId = actions.get(0).getCarId();
                    finalPlans.add(carId);
                    if (priorityRelation.higherPriority().contains(carId) && shouldReplan()) {
                        replan();
                    } else if (allHigherPriorityReady() && id != carId && !planSent) {
                        actuator.act(agentReact());
                    }
                }
            }
        });
    }

    private boolean replanTime() {
        return ((int) time) % MAX_TIME == 0;
    }

    private boolean shouldReplan() {
        if (allHigherPriorityReady() && (trajectory == null || replanTime())) {
            return true;
        }
        return false;
    }
    private boolean allHigherPriorityReady() {
        for (int higherPriority: priorityRelation.higherPriority()) {
            if (!finalPlans.contains(higherPriority)) {
                return false;
            }
        }
        return true;
    }

    public long getExpandedNodes() {
        return expanded;
    }

    private void replan() {
        RoadObject me = sensor.senseCurrentState(); // my current state
        Point2d pos2D = new Point2d(me.getPosition().x, me.getPosition().y);

        // Goal not reached yet
//        if (goal == null || pos2D.distance(goal) > 1) {

//            if (trajectory != null && time > trajectory.getMaxTime() - REPLAN_THRESHOLD) {
//                print("THRESHOLD REACHED, REPLANNING");
//                needsReplan = true;
//            }
//            if (needsReplan) {
            print("PLANNING");

            List<Region> movingObstacles = new LinkedList<Region>();

            // Add trajectories of higher priority agents to the moving obstacle list
            addTrajectoriesOfHigherPriority(movingObstacles);

            // Allow lower priority agents to perform safe maneuver
            allowSafeManeuver(movingObstacles);

//            for (Region obstacle: movingObstacles) {
//                MovingCircle mc = (MovingCircle) obstacle;
//                print(mc.getTrajectory().toString());
//            }
//                planningGraph = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, new int[] {speed}, new ArrayList<Region>(agentTrajectories), 1, 1);
//                planningGraph = new ConstantSpeedTimeExtension(spatialGraph, MAX_TIME, new int[]{speed}, movingObstacles, waitDuration, 1);
            planningGraph = new DynamicSpeedTimeExtension(spatialGraph, MAX_TIME + time, dynamicConstraint, movingObstacles, waitDuration);
            planningGraph = new ControlEffortWrapper(planningGraph, movePenalty, waitPenalty);
            // Do the planning
            this.plan();
            needsReplan = false;
            if (movingRegionLayer != null) {
                VisManager.unregisterLayer(movingRegionLayer);
            }
            if (trajectory != null) {
                movingRegionLayer = KeyToggleLayer.create("" + id, true, tt.euclidtime3i.vis.RegionsLayer.create(new tt.euclidtime3i.vis.RegionsLayer.RegionsProvider() {

                    @Override
                    public Collection<tt.euclidtime3i.Region> getRegions() {
                        List<tt.euclidtime3i.Region> regions = new ArrayList<tt.euclidtime3i.Region>(1);
                        regions.add(new tt.euclidtime3i.region.MovingCircle(new DiscreteTrajectoryWrapper(trajectory), RADIUS));
                        return regions;
                    }
                }, new TimeParameterProjectionTo2d(timeParameter), AgentColors.getColorForAgent(id), AgentColors.getColorForAgent(id)));
                VisManager.registerLayer(movingRegionLayer);
            }
//        }
//        }


    }

    private List<Action> agentReact() {
        RoadObject me = sensor.senseCurrentState(); // my current state
        LinkedList<Action> actions = new LinkedList<Action>(); // list of actions for the simulator
        SpeedPoint p = trajectory.get(time);
//                System.out.println("Trajectory [" + time + "] point ("+id+"): "+p);
        if (p == null) {
            //needsReplan = true;
            p = trajectory.get(trajectory.getMaxTime());
        }
        Point3f planP = new Point3f((float)p.x, (float)p.y, 0);
        actions.add(new WPAction(id, me.getUpdateTime(), planP, p.getSpeed()));
        planSent = true;
        return actions;
    }

    private void addTrajectoriesOfHigherPriority(List<Region> movingObstacles) {
        Map<Integer, Region> trajectories = sensor.senseTrajectories();
        List<Integer> higherPriority = priorityRelation.higherPriority();
        for (int id: higherPriority) {
            Region t = trajectories.get(id);
            if (t != null) {
                movingObstacles.add(t);
            }
        }
    }

    private void allowSafeManeuver(List<Region> movingObstacles) {
        List<Integer> lowerPriority = priorityRelation.lowerPriority();
        for (int id: lowerPriority) {
            RoadObject agent = sensor.senseCar(id);
            if (agent != null) {
                movingObstacles.add(generateSafeManeuverForAgent(agent));
            }
        }
    }

    /**
     * Generate trajectory simulating safe maneuver
     * @param agent
     * @return
     */
    private Region generateSafeManeuverForAgent(final RoadObject agent) {
        List<Straight> traj = new LinkedList<Straight>();
        Point3f agentPosition = agent.getPosition();
        SpeedPoint agentPoint = new SpeedPoint(agentPosition.x, agentPosition.y, 0);
        // Allow agent to stay on the current position
        for (double t = time; t < 2*MAX_TIME+time; ++t) {
            traj.add(new Straight(agentPoint, t, agentPoint, t+1));

        }
        Trajectory trajectory = new BasicSegmentedTrajectory(traj, 5*MAX_TIME+time);
        return new MovingCircle(trajectory, RADIUS);
    }


    /**
     * Plan the optimal non-collision trajectory
     */
    private void plan() {
        if (trajectoryLayer != null) {
            VisManager.unregisterLayer(trajectoryLayer);
        }
        RoadObject currentState = sensor.senseCurrentState();
        if (goal == null) {
            Edge lastEdge = navigator.getRoute().get(navigator.getRoute().size() - 1);
            Iterator<Lane> laneIterator = lastEdge.getLanes().values().iterator();
            Lane l = laneIterator.next();
            List<Point2f> innerPs = l.getInnerPoints();
            Point2f lastPoint = innerPs.get(innerPs.size() - 1);
            goal = new Point(lastPoint.x, lastPoint.y);
        }
        expanded = 0;

        HeuristicToGoal<Point4d> heuristicToGoal;
        DirectedGraph<Point, Line> reversed = new EdgeReversedGraph<Point, Line>(spatialGraph);
        heuristicToGoal = new ShortestPathHeuristic(reversed, goal, MAX_SPEED, MAX_ACC);
        timer.reset();
        double speed;
        Point start;
        speed = currentState.getVelocity().length();
        Point3f currentPosition = currentState.getPosition();
        start = new Point(currentPosition.x, currentPosition.y);
        if (!spatialGraph.containsVertex(start)) {
            start = findNearestGraphVertex(spatialGraph, start);
        }

        print("Start: "+start+", time: "+time);
        print("Goal: " + goal);
//        try {
//            System.in.read();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }

        planningGraph = new FreeOnTargetWaitExtension(planningGraph, goal);
        Point4d startPoint = new Point4d(start.x, start.y, speed, time);
        GraphPath<Point4d, Straight> path = AStarShortestPathSimple.findPathBetween(planningGraph,
                heuristicToGoal,
            startPoint, new Goal<Point4d>() {
                @Override
                public boolean isGoal(Point4d point) {
                    ++expanded;
//                    print("Trying: "+point);
//                    return (point.getTime() >= MAX_TIME+time || goal.distance(point.getPosition()) < 1);
                    return (point.getTime() >= MAX_TIME+time);
//                    return (goal.distance(point.getPosition()) < 1);
                }
            });
        float planTime = timer.getRawElapsedTime();
        if (planTime > maxTime) {
            maxTime = planTime;
        }
        nOfReplans++;
        print("Path: "+path);
        print("Planning took: " + timer.getElapsedTime() + ", expanded nodes: " + expanded);
//        print("Maximal time: "+maxTime+", number of replans: "+nOfReplans);

        if (path == null) {
            trajectory = null;
            ++notFound;
            print("No path found! Sum: "+notFound);
            return;
        }
        print("Trajectory end time: "+path.getEndVertex().getTime());
        trajectory = new StraightSegmentTrajectory(path, path.getEndVertex().getTime()-time);
        MovingCircle region = new MovingCircle(trajectory, RADIUS);

        // Dispatch an event
        actuator.getEventProcessor().addEvent(HighwayEventType.TRAJECTORY_UPDATED, null, null,
                new AbstractMap.SimpleEntry<Integer, Region>(this.id, region));
//        trajectoryLayer = TrajectoryLayer.create(new TrajectoryLayer.TrajectoryProvider<Point>() {
//            @Override
//            public tt.continous.Trajectory<Point> getTrajectory() {
//                return trajectory;
//            }
//        }, new ProjectionTo2d(), AgentColors.getColorForAgent(id), 1, trajectory.getMaxTime(), 't');
//        VisManager.registerLayer(trajectoryLayer);
    }

    /**
     * Finds nearest point in graph to the on given
     */
    private <V extends Point2d, E> V findNearestGraphVertex(Graph<V, E> graph, V vertex) {
        double RADIUS = 1d;
        // TODO: use KD-tree
        for (V v: graph.vertexSet()) {
            if (v.distance(vertex) <= RADIUS) {
                return v;
            }
        }
        return null;
    }
    /**
     * Print if verbose is set
     */
    private void print(String s) {
        if (verbose) {
            System.out.println("[" + id + "][" + time + "]: " + s);
        }
    }
}
