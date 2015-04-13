package cz.agents.highway.environment.planning.dynamics;

import cz.agents.highway.environment.planning.euclid3d.SpeedPoint;
import cz.agents.highway.environment.planning.euclid4d.Point4d;
import tt.euclid2d.Point;

import javax.vecmath.Point2d;
import java.util.LinkedList;
import java.util.List;

/**
 * Simple DynamicConstraint implementation based on acceleration
 * Created by wmatex on 4.4.15.
 */
public class AccelerationDynamicConstraint implements DynamicConstraint {
    private final double maximalAcceleration;
    private final double maximalSpeed;

    public AccelerationDynamicConstraint(double maximalAcceleration, double maximalSpeed) {
        this.maximalAcceleration = maximalAcceleration;
        this.maximalSpeed = maximalSpeed;
    }

    @Override
    public List<Point4d> expand(final Point4d current, final Point toExpand) {
        List<Point4d> expansion = new LinkedList<Point4d>();
        // Keep current speed
        double distance = current.getPosition().distance(toExpand);
        double speed = current.getSpeed();
        double time = distance/speed;
        expansion.add(new Point4d(toExpand, speed, time+current.getTime()));

        // Accelerate
        double sqrt = speed*speed + 2*maximalAcceleration*distance;
        double t = (-speed+Math.sqrt(sqrt))/maximalAcceleration;
        double endSpeed = maximalAcceleration*t+speed;
        if (endSpeed > maximalSpeed) {
            endSpeed = maximalSpeed;
            t = 2*distance/(endSpeed+speed);
        }
        expansion.add(new Point4d(toExpand, endSpeed, t+current.getTime()));

        // Decelerate
        sqrt = speed*speed - 2*maximalAcceleration*distance;
        if (sqrt < 0) {
            endSpeed = 0;
            t = 2*distance/speed;
        } else {
            t = (-speed+Math.sqrt(sqrt))/(-maximalAcceleration);
            endSpeed = -maximalAcceleration*t+speed;
        }
        expansion.add(new Point4d(toExpand, endSpeed, t+current.getTime()));

        return expansion;
    }
}
