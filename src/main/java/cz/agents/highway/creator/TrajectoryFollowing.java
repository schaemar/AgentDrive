package cz.agents.highway.creator;

import cz.agents.highway.storage.plan.WPAction;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;
import geometry_msgs.Vector3;
import nav_msgs.Odometry;
import org.ros.message.MessageFactory;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import javax.vecmath.*;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Created by david on 7/10/15.
 * The code itself is mainly this c++ demo
 * https://syrotek.felk.cvut.cz/data/files/codes/dem_trajectory_following_2.tar.gzhttps://syrotek.felk.cvut.cz/data/files/codes/dem_trajectory_following_2.tar.gz
 * rewritten to the java
 */

public class TrajectoryFollowing {
    private final static long ANGLE_TO_VELOCITY = 1;
    private final static int APPROX_ITERATIONS = 10;
    private LinkedBlockingQueue<WPAction> trajectory;    //!<Defined trajectory.
    private Point2f lastRemoved;    //!<Last point, removed frome the queue.
    private Point2f target;        //!<Target point, which is actual destination of robot.
    private double tolerance;    //!<Distance from destination, which will be tolerated.
    private double plannedSpeed;        //!<Default plannedSpeed of robot.
    private double actualSpeed;         //actual robot speed
    private double targetDistance;    //!< Distance of target point on trajectory. (targetDistance >> tolerance)
    private ConnectedNode connectedNode; //to be able to create updates
    private Point2f actualP2F;          //actual position
    private Quaternion quaternion;      //actual rotation
    private Publisher<Twist> testPublisher;     //for sending updates
    private boolean goalAchieved = false;
    private final CountDownLatch latch = new CountDownLatch(1); //lock for thread syncing. Running Rosjava node takes some time.


    public TrajectoryFollowing(final ConnectedNode connectedNode, Publisher<Twist> testPublisher, LinkedBlockingQueue<WPAction> followedTrajectory, double tol, double sp, double targetDist) {

        trajectory = followedTrajectory;
        if (targetDist < 2 * tol) {
            System.out.println("target distance must be bigger than 2*tolerance");
            System.exit(1);
        }
        targetDistance = targetDist;
        tolerance = tol;
        plannedSpeed = sp;
        lastRemoved = point3ftoPoint2f(trajectory.peek().getPosition());
        plannedSpeed = trajectory.peek().getSpeed();
        if (plannedSpeed < 0) plannedSpeed = 0;
        target = new Point2f(1.0f, 1.0f);
        trajectory.poll();
        this.connectedNode = connectedNode;
        this.testPublisher = testPublisher;
        actualP2F = null;
    }

    private void publishMessage(double angleCommand, double speedCommand) {
        // $ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
        MessageFactory msgFactory = connectedNode.getTopicMessageFactory(); //factorty fro creating Twist messages
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

    //Subscriber
    public void messageCallback(Odometry msg) {
        double angleCommand = 0;
        double speedCommand = 0;
        actualP2F = new Point2f((float) msg.getPose().getPose().getPosition().getX(), (float) msg.getPose().getPose().getPosition().getY());
        quaternion = msg.getPose().getPose().getOrientation();
        actualSpeed = msg.getTwist().getTwist().getLinear().getX();
        // System.out.printf("ACTLINSPEED: %f %f %f\n",msg.getTwist().getTwist().getLinear().getX(),msg.getTwist().getTwist().getLinear().getY(),msg.getTwist().getTwist().getLinear().getZ());
        latch.countDown();
        //quaternion = new Point4f((float)qutemp.getX(),(float)qutemp.getY(),(float)qutemp.getZ(),(float)qutemp.getW());
        findTarget(actualP2F); //some magic

        if (closeEnough(actualP2F) == true && trajectory.isEmpty()) {
            //  System.out.println("GOAL ACHIEVED");
            goalAchieved = true;
        }
        if (goalAchieved) {
            angleCommand = 0.0;
            speedCommand = 0.0;
        } else {
            speedCommand = calculateSpeed(actualP2F);
            angleCommand = calculateAngle(actualP2F, 2.0 * Math.asin(msg.getPose().getPose().getOrientation().getZ()), plannedSpeed);
        }
        //Invoking method for publishing message
        //publishMessage(angleCommand, speedCommand);

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
        return plannedSpeed;
    }

    private void findTarget(Point2f actual) {
        // Removing points from the queue until point with higher distance
        // from robot than targetDistance is found.

        if (trajectory.isEmpty() == false) {
            System.out.println();
            while (actual.distance(point3ftoPoint2f(trajectory.peek().getPosition())) < targetDistance) {
                lastRemoved = point3ftoPoint2f(trajectory.peek().getPosition());
                plannedSpeed = trajectory.peek().getSpeed();
                if (plannedSpeed < 0) plannedSpeed = 0;
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
            for (int i = 1; i < APPROX_ITERATIONS; i++) {

                Point2f stimes = new Point2f((float) (Math.pow(2, -i) * s.getX()), (float) (Math.pow(2, -i) * s.getX()));
                if (getAbs(plusplusplusminus(lastRemoved, stimes, v, actual)) < targetDistance) {
                    v.setX(v.getX() + (float) Math.pow(2, -i) * s.getX());
                    v.setY(v.getY() + (float) Math.pow(2, -i) * s.getY());
                }
            }
            target.setX(lastRemoved.getX() + v.getX());
            target.setY(lastRemoved.getY() + v.getY());
        }


    }

    double getAbs(Point2f point) {
        return Math.sqrt((Math.pow(point.getY(), 2.0)) + (Math.pow(point.getX(), 2.0)));
    }

    private Point2f plusplusplusminus(Point2f a, Point2f b, Point2f c, Point2f d) {
        return new Point2f(a.getX() + b.getX() + c.getX() - d.getX(), a.getY() + b.getY() + c.getY() - d.getY());
    }

    private Point2f point3ftoPoint2f(Point3f point) {
        return new Point2f(point.getX(), point.getY());
    }

    public void setNewPlan(LinkedBlockingQueue<WPAction> trajectory) {
        synchronized (this) {
            this.trajectory = trajectory;
            lastRemoved = point3ftoPoint2f(trajectory.peek().getPosition());
            plannedSpeed = trajectory.peek().getSpeed();
            if (plannedSpeed < 0) plannedSpeed = 0;
            trajectory.poll();
            goalAchieved = false;
        }

    }

    public double getActualSpeed() throws InterruptedException {
        latch.await();
        return actualSpeed;
    }

    public Point2f getTarget() throws InterruptedException {
        latch.await();
        return target;
    }

    public synchronized Point2f getActualP2F() throws InterruptedException {
        latch.await();
        return actualP2F;
    }

    private double QuadrationToAngle_now(double sqw, double sqz) {
        int pom = 0;
        if (sqw < 0) {
            pom = 1;
            sqw = -sqw;
        }
        if (sqz < 0) {
            pom = 1;
        }
        double result;
        double angle = Math.acos(sqw) * 2;
        if (pom == 1) {
            result = -angle;
        } else {
            result = angle;
        }
        return result;
    }

    private Vector2f angleToVelocityVector(double angle) {
        if(angle <0) angle = 6.2831853072 + angle;
        Vector2f velVec = new Vector2f((float) (Math.cos(angle)), (float) (Math.sin(angle)));
        velVec.normalize();
        return velVec;
    }
    public Vector2f getActualVelocityVector()
    {
        return angleToVelocityVector(QuadrationToAngle_now(quaternion.getW(),quaternion.getZ()));
    }
}
