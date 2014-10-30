package cz.agents.highway.environment.planning.graph;

import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.graph.GraphDelegator;
import tt.euclidtime3i.Point;
import tt.euclidtime3i.discretization.Straight;

/**
 * Weight the edges of the planning graph based on distance of the edge
 * Created by wmatex on 29.10.14.
 */
public class ControlEffortWrapper extends GraphDelegator<Point, Straight> implements DirectedGraph<Point, Straight> {
    public ControlEffortWrapper(Graph<Point, Straight> g) {
        super(g);
    }

    @Override
    public double getEdgeWeight(Straight e) {
        return e.getStart().getPosition().distance(e.getEnd().getPosition());
    }
}
