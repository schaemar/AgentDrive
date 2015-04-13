package cz.agents.highway.environment.planning.graph;

import cz.agents.highway.environment.planning.euclid4d.Point4d;
import cz.agents.highway.environment.planning.euclid4d.Straight;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.graph.GraphDelegator;

/**
 * Weight the edges of the planning graph based on distance of the edge
 * Created by wmatex on 29.10.14.
 */
public class ControlEffortWrapper extends GraphDelegator<Point4d, Straight> implements DirectedGraph<Point4d, Straight> {
    private double movePenaltyPerSec;
    private double waitPenaltyPerSec;

    public ControlEffortWrapper(Graph<Point4d, Straight> g, double movePenaltyPerSec, double waitPenaltyPerSec) {
        super(g);
        this.movePenaltyPerSec = movePenaltyPerSec;
        this.waitPenaltyPerSec = waitPenaltyPerSec;
    }

    @Override
    public double getEdgeWeight(Straight e) {
        if (e.getStart().getPosition().distance(e.getEnd().getPosition()) < 0.01) {
//            return waitPenaltyPerSec * (e.duration());
            return super.getEdgeWeight(e);
        } else {
            return super.getEdgeWeight(e) + movePenaltyPerSec * (e.duration());
//            return e.duration();
//            return super.getEdgeWeight(e);
        }
    }
}
