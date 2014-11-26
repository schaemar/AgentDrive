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
    private double movePenaltyPerSec;
    private double waitPenaltyPerSec;

    public ControlEffortWrapper(Graph<Point, Straight> g, double movePenaltyPerSec, double waitPenaltyPerSec) {
        super(g);
        this.movePenaltyPerSec = movePenaltyPerSec;
        this.waitPenaltyPerSec = waitPenaltyPerSec;
    }

    @Override
    public double getEdgeWeight(Straight e) {
        if (e.getStart().getPosition().distance(e.getEnd().getPosition()) < 0.01) {
            return super.getEdgeWeight(e) + waitPenaltyPerSec * (e.duration());
        } else {
            return super.getEdgeWeight(e) + movePenaltyPerSec * (e.duration());
        }
    }
}
