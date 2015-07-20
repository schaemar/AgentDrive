package cz.agents.highway.creator;

import cz.agents.highway.storage.plan.WPAction;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;
import geometry_msgs.Vector3;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Point4f;
import javax.vecmath.Vector2f;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Created by david on 7/10/15.
 */

public class TrajectoryFollowing {
    private final static long ANGLE_TO_VELOCITY = 1;
    private final static int APPROX_ITERATIONS = 10;
    LinkedBlockingQueue<WPAction> trajectory;    //!<Defined trajectory.
    Point2f lastRemoved;    //!<Last point, removed frome the queue.
    Point2f target;        //!<Target point, which is actual destination of robot.
    double tolerance;    //!<Distance from destination, which will be tolerated.
    double speed;        //!<Default speed of robot.
    double targetDistance;    //!< Distance of target point on trajectory. (targetDistance >> tolerance)
    private ConnectedNode connectedNode;
    private Point2f actualP2F;
    private Point4f quaternion;
    Publisher<Twist> testPublisher;
    private boolean goalAchieved = false;

    public Point4f getQuaternion() {
        return quaternion;
    }

    public TrajectoryFollowing(final ConnectedNode connectedNode, Publisher<Twist> testPublisher, LinkedBlockingQueue<WPAction> followedTrajectory, double tol, double sp, double targetDist) {

        trajectory = followedTrajectory;
        if (targetDist < 2 * tol) {
            System.out.println("target distance must be bigger than 2*tolerance");
            System.exit(1);
        }
        targetDistance = targetDist;
        tolerance = tol;
        speed = sp;
        lastRemoved = point3ftoPoint2f(trajectory.peek().getPosition());
        target = new Point2f(1.0f, 1.0f);
        trajectory.poll();
        this.connectedNode = connectedNode;
        this.testPublisher = testPublisher;
        actualP2F = null;
    }

    private void publishMessage(double angleCommand, double speedCommand) {
        // $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
        MessageFactory msgFactory = connectedNode.getTopicMessageFactory();
        std_msgs.Header header = msgFactory.newFromType(std_msgs.Header._TYPE);

        Twist msg = testPublisher.newMessage();

        header.setFrameId("bagr");
        Vector3 linear = msgFactory
                .newFromType(Vector3._TYPE);
        Vector3 angular = msgFactory
                .newFromType(Vector3._TYPE);
        linear.setX(speedCommand);
        angular.setZ(angleCommand * ANGLE_TO_VELOCITY);
        msg.setAngular(angular);
        msg.setLinear(linear);
        testPublisher.publish(msg);
    }

    public synchronized Point2f getActualP2F() {
        return actualP2F;
    }

    //Subscriber
    public void messageCallback(Odometry msg) {
        double angleCommand = 0;
        double speedCommand = 0;
        actualP2F = new Point2f((float) msg.getPose().getPose().getPosition().getX(), (float) msg.getPose().getPose().getPosition().getY());
        Quaternion qutemp = msg.getPose().getPose().getOrientation();
        quaternion = new Point4f((float)qutemp.getX(),(float)qutemp.getY(),(float)qutemp.getZ(),(float)qutemp.getW());
        findTarget(actualP2F);

        if (closeEnough(actualP2F) == true && trajectory.isEmpty()) {
            System.out.println("GOAL ACHIEVED");
            goalAchieved = true;
        }

        speedCommand = calculateSpeed(actualP2F);
        angleCommand = calculateAngle(actualP2F, 2.0 * Math.asin(msg.getPose().getPose().getOrientation().getZ()), speed);
        //Invoking method for publishing message
        //publishMessage(angleCommand, speedCommand);
    /*    if(goalAchieved)
        {
            angleCommand = 0.0;
            speedCommand = 0.0;
        }*/
        publishMessage(angleCommand, speedCommand);
    }

    double calculateAngle(Point2f actual, double angle, double speed) {
        double angleCalc;
        double distance;
        double radius;
        double routeDistance;
        double time;
        double angleVelocity;
        
       // angleCalc = actual -> getAngle(target) - angle;
        angleCalc = Math.atan2(target.getY() - actual.getY(), target.getX() - actual.getX()) - angle;
        //normalizing angle to <-pi; pi>
        if (Math.abs(angleCalc) > Math.PI) {
            angleCalc = angleCalc - Math.copySign(2 * Math.PI, angleCalc);
        }
        distance = actual.distance(target);//      actual -> getDistance(target);
        radius = Math.abs((distance / 2) / (Math.cos(Math.PI / 2 - angleCalc)));
        routeDistance = 2 * Math.PI * radius * (Math.abs(angleCalc) / (Math.PI));
        time = routeDistance / speed;
        angleVelocity = 2 * angleCalc / time;
        return angleVelocity;
    }

    private boolean closeEnough(Point2f actual) {
        double distance;
        distance = actual.distance(target);
        if (distance > tolerance) {
            return false;
        }
        return true;
    }

    private double calculateSpeed(Point2f actual) {
        return speed;
    }

    private void findTarget(Point2f actual) {
        // Removing points from the queue until point with higher distance
        // from robot than targetDistance is found.

        if (trajectory.isEmpty() == false) {
            while (actual.distance(point3ftoPoint2f(trajectory.peek().getPosition())) < targetDistance){
                lastRemoved = point3ftoPoint2f(trajectory.peek().getPosition());
                trajectory.poll();
                if (trajectory.isEmpty() == true) {
                    break;
                }
            }
        }
        if (trajectory.isEmpty() == true) {
            target.setX(lastRemoved.getX());
            target.setY(lastRemoved.getY());
        } else {
            //circle - line aproximation

            //vector: FRONT - LAST REMOVED
            Point2f s = new Point2f(point3ftoPoint2f(trajectory.peek().getPosition()).getX() - lastRemoved.getX(), point3ftoPoint2f(trajectory.peek().getPosition()).getY()
            - lastRemoved.getY());

            //vector which will be added to lastRemoved
            Point2f v = new Point2f(0.0f, 0.0f);

            if (actual.distance(lastRemoved) >= targetDistance && actual.distance(point3ftoPoint2f(trajectory.peek().getPosition())) >= targetDistance &&
                    (lastRemoved.getX() != point3ftoPoint2f(trajectory.peek().getPosition()).getX() || lastRemoved.getY() != point3ftoPoint2f(trajectory.peek().getPosition()).getY()))    //two intersections
            {
                //finding point between intersections
                //calculating line
                double a, b, c, dist;
                a = lastRemoved.getY() - point3ftoPoint2f(trajectory.peek().getPosition()).getY();
                b = point3ftoPoint2f(trajectory.peek().getPosition()).getX() - lastRemoved.getX();
                c = lastRemoved.getX() * point3ftoPoint2f(trajectory.peek().getPosition()).getY() - point3ftoPoint2f(trajectory.peek().getPosition()).getX() * lastRemoved.getY();
                if (a * a + b * b < 0.001)    //front and lastRemoved are the same
                {
                    target.setX(lastRemoved.getX());
                    target.setY(lastRemoved.getY());
                    return;
                }
                dist = Math.abs(a * actual.getX() + b * actual.getY() + c) / Math.sqrt(a * a + b * b);    //distance between point and line
                double distFromLast;
                distFromLast = Math.sqrt(Math.pow(lastRemoved.distance(actual), 2.0) - Math.pow(dist, 2.0));
                //s.setX((float) distFromLast / s.getX());
                v.setX((float)distFromLast/(float)getAbs(s) * s.getX());
                //s.setY((float) distFromLast/s.getY());
                v.setY((float)distFromLast/(float) getAbs(s) * s.getY());

                s.setX(s.getX() - v.getX());
                s.setY(s.getY() - v.getY());
              //  v -> x = (distFromLast/s -> getAbs())*s -> x;
              //  v -> y = (distFromLast/s -> getAbs())*s -> y;
              //  s -> x = s -> x - v -> x;
              //  s -> y = s -> y - v -> y;
            }
            for (int i = 1; i < APPROX_ITERATIONS; i++) {

                Point2f stimes = new Point2f((float)(Math.pow(2,-i) * s.getX()),(float)(Math.pow(2,-i) * s.getX()));
                if (getAbs(plusplusplusminus(lastRemoved, stimes, v, actual)) < targetDistance)
                {
                   // *v =*v + s -> times(pow(2, -i));
                   v.setX(v.getX() + (float)Math.pow(2,-i) * s.getX());
                   v.setY(v.getY() + (float)Math.pow(2,-i) * s.getY());
                }

            }
            target.setX(lastRemoved.getX() + v.getX());
            target.setY(lastRemoved.getY() + v.getY());
           // target -> x = lastRemoved -> x + v -> x;
           // target -> y = lastRemoved -> y + v -> y;
        }


    }

    double getAbs(Point2f point)
    {
        return Math.sqrt((Math.pow(point.getY(),2.0)) + (Math.pow(point.getX(),2.0)));
    }
    private Point2f plusplusplusminus(Point2f a, Point2f b,Point2f c,Point2f d)
    {
        return new Point2f(a.getX() + b.getX() + c.getX() - d.getX(),a.getY() + b.getY() +c.getY() - d.getY());
    }
    private Point2f point3ftoPoint2f(Point3f point)
    {
        return new Point2f(point.getX(),point.getY());
    }

    public void setTrajectory(LinkedBlockingQueue<WPAction> trajectory) {
        this.trajectory = trajectory;
    }
    public Point2f getPosition()
    {
        return actualP2F;
    }

    public double getSpeed() {
        return speed;
    }

    public Point2f getTarget() {
        return target;
    }
}
