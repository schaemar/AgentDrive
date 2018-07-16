package cz.agents.agentdrive.highway.environment.planning.graph;

import cz.agents.agentdrive.highway.environment.planning.euclid4d.Point4d;
import cz.agents.agentdrive.highway.environment.planning.euclid4d.Straight;
import org.jgrapht.DirectedGraph;
import org.jgrapht.Graph;
import org.jgrapht.graph.GraphDelegator;

@SuppressWarnings("serial")
public class FreeOnTargetWaitExtension extends GraphDelegator<Point4d, Straight> implements DirectedGraph<Point4d, Straight> {

	tt.euclid2d.Point targetVertex;

    public FreeOnTargetWaitExtension(Graph<Point4d, Straight> g, tt.euclid2d.Point targetVertex) {
        super(g);
        this.targetVertex = targetVertex;
    }

	@Override
    public double getEdgeWeight(Straight e) {
        if (e.getStart().getPosition().distance(targetVertex) < 0.001 && e.getEnd().getPosition().distance(targetVertex) < 0.001) {
            return 0;
        } else {
            return super.getEdgeWeight(e);
        }
    }
}
