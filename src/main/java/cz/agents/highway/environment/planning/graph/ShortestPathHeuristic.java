package cz.agents.highway.environment.planning.graph;

import cz.agents.highway.environment.planning.euclid4d.Point4d;
import org.jgrapht.Graph;
import org.jgrapht.util.HeuristicToGoal;
import org.jgrapht.util.heuristics.PerfectHeuristic;
import tt.euclid2d.Line;
import tt.euclid2d.Point;


public class ShortestPathHeuristic implements HeuristicToGoal<Point4d> {

    private HeuristicToGoal<Point> heuristics;
    private final double maxSpeed;
    private final double maxAcceleration;
    private final double maxTime;
    private final double PENALTY = 10;

    public ShortestPathHeuristic(Graph<Point, Line> graph, Point target, double maxSpeed, double maxAcceleration, double maxTime) {
        this.heuristics = new PerfectHeuristic<Point, Line>(graph, target);
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
        this.maxTime = maxTime;
    }

    @Override
    public double getCostToGoalEstimate(Point4d current) {
        //return current.getTime() + heuristics.getCostToGoalEstimate(current.getPosition())/maxSpeed
        //        + current.getActualSpeed()/maxAcceleration;
        double heuristicsEstimate = heuristics.getCostToGoalEstimate(current.getPosition());
//        if (current.getTime() + current.getActualSpeed()/maxAcceleration > maxTime+maxSpeed/maxAcceleration) {
////            return heuristicsEstimate + PENALTY*(maxTime - current.getTime()-current.getActualSpeed()/maxAcceleration);
//            return Double.POSITIVE_INFINITY;
//        } else {
//            return heuristicsEstimate;
//        }

        return heuristicsEstimate;
//        double distance = heuristics.getCostToGoalEstimate(current.getPosition());
//        double timeToMaxSpeed = (maxSpeed - current.getActualSpeed())/maxAcceleration;
//        double distanceTraveled = 0.5*maxAcceleration*timeToMaxSpeed*timeToMaxSpeed+current.getActualSpeed()*timeToMaxSpeed;
//        return timeToMaxSpeed + (distance-distanceTraveled)/maxSpeed;
    }
}
