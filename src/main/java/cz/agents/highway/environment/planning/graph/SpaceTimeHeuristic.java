package cz.agents.highway.environment.planning.graph;

import org.jgrapht.util.HeuristicToGoal;
import tt.euclidtime3i.Point;

import javax.vecmath.Point2i;

/**
 * Created by wmatex on 18.11.14.
 */
public class SpaceTimeHeuristic implements HeuristicToGoal<Point> {
    private tt.euclid2i.Point goal, start;
    public SpaceTimeHeuristic(tt.euclid2i.Point goal, tt.euclid2i.Point start) {
        this.goal = goal;
        this.start = start;
    }
    @Override
    public double getCostToGoalEstimate(Point current) {
        double speed = current.getPosition().distance(start)/(current.getTime()+1);
//        return current.getPosition().distance(goal)/speed;
        return current.getPosition().distance(goal);
    }
}
