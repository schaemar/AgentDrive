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

    public ShortestPathHeuristic(Graph<Point, Line> graph, Point target, double maxSpeed, double maxAcceleration) {
        this.heuristics = new PerfectHeuristic<Point, Line>(graph, target);
        this.maxSpeed = maxSpeed;
        this.maxAcceleration = maxAcceleration;
    }

    @Override
    public double getCostToGoalEstimate(Point4d current) {
        //return current.getTime() + heuristics.getCostToGoalEstimate(current.getPosition())/maxSpeed
        //        + current.getSpeed()/maxAcceleration;
        return heuristics.getCostToGoalEstimate(current.getPosition());

//        double distance = heuristics.getCostToGoalEstimate(current.getPosition());
//        double timeToMaxSpeed = (maxSpeed - current.getSpeed())/maxAcceleration;
//        double distanceTraveled = 0.5*maxAcceleration*timeToMaxSpeed*timeToMaxSpeed+current.getSpeed()*timeToMaxSpeed;
//        return timeToMaxSpeed + (distance-distanceTraveled)/maxSpeed;
    }
}
