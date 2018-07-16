package cz.agents.agentdrive.highway.environment.planning.graph;

import ags.utils.dataStructures.DistanceFunction;
import ags.utils.dataStructures.KdTree;
import ags.utils.dataStructures.SquareEuclideanDistanceFunction;
import ags.utils.dataStructures.utils.MaxHeap;
import cz.agents.agentdrive.highway.agent.ADPPAgent;
import cz.agents.agentdrive.highway.environment.planning.dynamics.AccelerationDynamicConstraint;
import cz.agents.agentdrive.highway.environment.planning.dynamics.DynamicConstraint;
import cz.agents.agentdrive.highway.environment.planning.euclid2d.BasicSegmentedTrajectory;
import cz.agents.agentdrive.highway.environment.planning.euclid4d.Point4d;
import cz.agents.agentdrive.highway.environment.planning.euclid4d.Region;
import cz.agents.agentdrive.highway.environment.planning.euclid4d.Straight;
import cz.agents.agentdrive.highway.environment.planning.euclid4d.region.MovingCircle;
import org.jgrapht.DirectedGraph;
import org.jgrapht.EdgeFactory;
import org.jgrapht.graph.AbstractDirectedGraphWrapper;
import tt.euclid2d.Line;
import tt.euclid2d.Point;

import java.util.Collection;
import java.util.HashSet;
import java.util.Set;

/**
 *
 * Created by wmatex on 3.4.15.
 */
public class DynamicSpeedTimeExtension extends AbstractDirectedGraphWrapper<Point4d, Straight> {

    protected DirectedGraph<Point, Line> spatialGraph;
    protected DynamicConstraint dynamicConstraint;
    protected double maxTime;
    protected double waitMoveDuration;
    protected Collection<? extends Region> dynamicObstacles;
    private KdTree<Straight> kdTree = new KdTree<Straight>(3);
    private DistanceFunction distanceFunction = new SquareEuclideanDistanceFunction();
    private double RADIUS;

    public DynamicSpeedTimeExtension(
            DirectedGraph<Point, Line> spatialGraph, double maxTime, DynamicConstraint dynamics,
            Collection<? extends Region> dynamicObstacles, double waitMoveDuration) {
        super();
        this.spatialGraph = spatialGraph;
        this.dynamicConstraint = dynamics;
        this.maxTime = maxTime;
        this.dynamicObstacles = dynamicObstacles;
        this.waitMoveDuration = waitMoveDuration;

//        buildKdTree();
    }

    @Override
    public Straight getEdge(Point4d start, Point4d end) {
        return new Straight(start, end);
    }

    @Override
    public EdgeFactory<Point4d, Straight> getEdgeFactory() {
        return null;
    }

    @Override
    public Point4d getEdgeSource(Straight edge) {
        return edge.getStart();
    }

    @Override
    public Point4d getEdgeTarget(Straight edge) {
        return edge.getEnd();
    }

    @Override
    public double getEdgeWeight(Straight edge) {
//        return edge.getEnd().getPosition().distance(edge.getStart().getPosition());
        return edge.getEnd().getTime() - edge.getStart().getTime();
    }


    @Override
    public Set<Straight> outgoingEdgesOf(Point4d vertex) {
        if (vertex.getTime() < maxTime) {
            Set<Straight> edges = new HashSet<Straight>();

            ADPPAgent.TIMER.startLoop();
            Set<Line> spatialEdges = spatialGraph.outgoingEdgesOf(new tt.euclid2d.Point(vertex.x, vertex.y));
            for (Line spatialEdge : spatialEdges) {
                ADPPAgent.TIMER.startExpansion();
                Set<Point4d> expanded = dynamicConstraint.expand(vertex, spatialEdge.getEnd());
                ADPPAgent.TIMER.measureExpansion();
                for (Point4d child: expanded) {
                    if (isVisible(vertex, child, dynamicObstacles)) {
//                    if (valid(vertex, child)) {
                        edges.add(new Straight(vertex, child));
                    }
                }
            }
            ADPPAgent.TIMER.measureLoop();



            // Allow waiting edge only if vehicle is not moving
            ADPPAgent.TIMER.startWaiting();
            if (vertex.getSpeed() < AccelerationDynamicConstraint.MIN_SPEED) {
                double endTime = vertex.getTime() + waitMoveDuration;
                Point4d waitingPoint = new Point4d(vertex.getPosition(), 0d, endTime);
                if (isVisible(vertex, waitingPoint, dynamicObstacles)) {
//                if (valid(vertex, waitingPoint)) {
                    edges.add(new Straight(vertex, waitingPoint));
                }
            }
            ADPPAgent.TIMER.measureWaiting();

            return edges;
        } else {
            return new HashSet<Straight>();
        }
    }

    private void buildKdTree() {
        for (Region region: dynamicObstacles) {
            MovingCircle movingCircle = (MovingCircle) region;
            RADIUS = ((MovingCircle) region).getRadius();
            BasicSegmentedTrajectory segmentedTrajectory = (BasicSegmentedTrajectory) movingCircle.getTrajectory();
            for (Straight straight: segmentedTrajectory.getSegments()) {
                double[] point = new double[3];
                Point4d start = straight.getStart();
                point[0] = start.x;
                point[1] = start.y;
                point[2] = start.getTime();
                kdTree.addPoint(point, straight);

                Point4d end = straight.getEnd();
                point = new double[3];
                point[0] = end.x;
                point[1] = end.y;
                point[2] = end.getTime();
                kdTree.addPoint(point, straight);
            }
        }
    }

    private boolean isVisible(Point4d start, Point4d end, Collection<? extends Region> obstacles) {
        ADPPAgent.TIMER.startObstacles();
        for (Region obstacle : obstacles) {
            if (obstacle.intersectsLine(start, end)) {
                ADPPAgent.TIMER.measureObstacles();
                return false;
            }
        }
        ADPPAgent.TIMER.measureObstacles();
        return true;
    }

    private boolean valid(Point4d start, Point4d end) {
//        MaxHeap<Straight> heap = kdTree.findNearestNeighbors(new double[]{start.x, start.y, start.getTime()}, 1, distanceFunction);
//        Straight nearest = heap.getMax();
//        if (start.getTime() > nearest.getStart().getTime() && start.getTime() < nearest.getEnd().getTime()) {
//            Point4d interpolate = nearest.interpolate(start.getTime());
//            if (start.getPosition().distance(interpolate.getPosition()) < 2 * RADIUS) {
//                return false;
//            }
//        }

        MaxHeap<Straight> heap = kdTree.findNearestNeighbors(new double[]{end.x, end.y, end.getTime()}, 1, distanceFunction);
        Straight nearest = heap.getMax();
        if (end.getTime() > nearest.getStart().getTime() && end.getTime() < nearest.getEnd().getTime()) {
            Point4d interpolate = nearest.interpolate(end.getTime());
            if (start.getPosition().distance(interpolate.getPosition()) < 2 * RADIUS) {
                return false;
            }
        }

        return true;
    }
}
