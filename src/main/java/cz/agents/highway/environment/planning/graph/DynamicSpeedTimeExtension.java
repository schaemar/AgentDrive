package cz.agents.highway.environment.planning.graph;

import cz.agents.highway.environment.planning.dynamics.DynamicConstraint;
import cz.agents.highway.environment.planning.euclid4d.Point4d;
import cz.agents.highway.environment.planning.euclid4d.Region;
import cz.agents.highway.environment.planning.euclid4d.Straight;
import org.jgrapht.DirectedGraph;
import org.jgrapht.EdgeFactory;
import org.jgrapht.graph.AbstractDirectedGraphWrapper;
import tt.euclid2d.Line;
import tt.euclid2d.Point;

import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 *
 * Created by wmatex on 3.4.15.
 */
public class DynamicSpeedTimeExtension extends AbstractDirectedGraphWrapper<Point4d, Straight> {

    private static final double TIME_TOLERANCE = 1d;
    protected DirectedGraph<Point, Line> spatialGraph;
    protected DynamicConstraint dynamicConstraint;
    protected double maxTime;
    protected double waitMoveDuration;
    protected Collection<? extends Region> dynamicObstacles;

    public DynamicSpeedTimeExtension(
            DirectedGraph<Point, Line> spatialGraph, double maxTime, DynamicConstraint dynamics,
            Collection<? extends Region> dynamicObstacles, double waitMoveDuration) {
        super();
        this.spatialGraph = spatialGraph;
        this.dynamicConstraint = dynamics;
        this.maxTime = maxTime;
        this.dynamicObstacles = dynamicObstacles;
        this.waitMoveDuration = waitMoveDuration;
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
            Set<Point4d> children = new HashSet<Point4d>();
            Set<Straight> edges = new HashSet<Straight>();

            Set<Line> spatialEdges = spatialGraph.outgoingEdgesOf(new tt.euclid2d.Point(vertex.x, vertex.y));
            for (Line spatialEdge : spatialEdges) {
                List<Point4d> expanded = dynamicConstraint.expand(vertex, spatialEdge.getEnd());
                for (Point4d child: expanded) {
//                    if (child.getTime() <= maxTime+TIME_TOLERANCE && isVisible(vertex, child, dynamicObstacles)) {
                    if (isVisible(vertex, child, dynamicObstacles)) {
                        edges.add(new Straight(vertex, child));
//                        children.add(child);
                    }
                }
            }



            // Allow waiting edge only if vehicle is not moving
            if (vertex.getSpeed() < 0.00001) {
                double endTime = vertex.getTime() + waitMoveDuration;
//                if (endTime > maxTime+TIME_TOLERANCE) {
//                    endTime = maxTime+TIME_TOLERANCE;
//                }
                Point4d waitingPoint = new Point4d(vertex.getSpeedPoint(), endTime);
                if (isVisible(vertex, waitingPoint, dynamicObstacles)) {
                    edges.add(new Straight(vertex, waitingPoint));
                }
            }

//            for (Point child : children) {
//                edges.add(new Straight(vertex, child));
//            }

            return edges;
        } else {
            return new HashSet<Straight>();
        }
    }

    private boolean isVisible(Point4d start, Point4d end, Collection<? extends Region> obstacles) {
        for (Region obstacle : obstacles) {
            if (obstacle.intersectsLine(start, end)) {
                return false;
            }
        }
        return true;
    }}
