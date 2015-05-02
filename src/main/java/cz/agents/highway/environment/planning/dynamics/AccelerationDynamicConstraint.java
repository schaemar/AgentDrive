package cz.agents.highway.environment.planning.dynamics;

import cz.agents.highway.environment.planning.euclid3d.SpeedPoint;
import cz.agents.highway.environment.planning.euclid4d.Point4d;
import tt.euclid2d.Point;

import javax.vecmath.Point2d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

/**
 * Simple DynamicConstraint implementation based on acceleration
 * Created by wmatex on 4.4.15.
 */
public class AccelerationDynamicConstraint implements DynamicConstraint {
    public static final double SPEED_TOLERANCE = 0.00001d;
    public static final double MIN_SPEED = 0.1d;
    private final double maximalAcceleration;
    private final double maximalSpeed;

    public AccelerationDynamicConstraint(double maximalAcceleration, double maximalSpeed) {
        this.maximalAcceleration = maximalAcceleration;
        this.maximalSpeed = maximalSpeed;
    }

    @Override
    public Set<Point4d> expand(final Point4d current, final Point toExpand) {
        Set<Point4d> expansion = new HashSet<Point4d>();
        double distance = current.getPosition().distance(toExpand);
        double speed = current.getSpeed();
        double time = distance / speed;
        if (current.getSpeed() >= MIN_SPEED) {
            // Keep current speed
            expansion.add(new Point4d(toExpand, speed, time + current.getTime()));
            // Decelerate
            double sqrt = speed*speed - 2*maximalAcceleration*distance;
            double endSpeed, t;
            if (sqrt < 0) {
                endSpeed = 0;
                t = 2*distance/speed;
            } else {
                t = (-speed+Math.sqrt(sqrt))/(-maximalAcceleration);
                endSpeed = -maximalAcceleration*t+speed;
            }
            expansion.add(new Point4d(toExpand, endSpeed, t+current.getTime()));
        }

        // Accelerate
        double sqrt = speed*speed + 2*maximalAcceleration*distance;
        double t = (-speed+Math.sqrt(sqrt))/maximalAcceleration;
        double endSpeed = maximalAcceleration*t+speed;
        if (endSpeed > maximalSpeed) {
            endSpeed = maximalSpeed;
            t = 2*distance/(endSpeed+speed);
        }
        expansion.add(new Point4d(toExpand, endSpeed, t+current.getTime()));

        return expansion;
    }
}
