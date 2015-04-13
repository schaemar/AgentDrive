package cz.agents.highway.environment.planning.euclid2d;

import tt.euclid2i.*;

import javax.vecmath.Point2d;

/**
 * Make continuous trajectory look like discrete one, to be able to integrate it easily with trajectory tools
 * Created by wmatex on 4.4.15.
 */
public class DiscreteTrajectoryWrapper implements tt.euclid2i.Trajectory {
    private final Trajectory continuousTrajectory;
    public DiscreteTrajectoryWrapper(Trajectory trajectory) {
        continuousTrajectory = trajectory;
    }
    @Override
    public int getMinTime() {
        return (int) Math.ceil(continuousTrajectory.getMinTime());
    }

    @Override
    public int getMaxTime() {
        return (int) Math.floor(continuousTrajectory.getMaxTime());
    }

    @Override
    public Point get(int t) {
        Point2d point2d = continuousTrajectory.get((double)t).getPosition();
        return new Point((int) Math.floor(point2d.x), (int) Math.floor(point2d.y));
    }
}
