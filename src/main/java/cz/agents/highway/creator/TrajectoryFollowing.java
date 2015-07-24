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
 */

public class TrajectoryFollowing {
    private final static long ANGLE_TO_VELOCITY = 1;
    private final static int APPROX_ITERATIONS = 10;
    private LinkedBlockingQueue<WPAction> trajectory;    //!<Defined trajectory.
    private Point2f lastRemoved;    //!<Last point, removed frome the queue.
    private Point2f target;        //!<Target point, which is actual destination of robot.
    private double tolerance;    //!<Distance from destination, which will be tolerated.
    private double plannedSpeed;        //!<Default plannedSpeed of robot.
    private double actualSpeed;
    private double targetDistance;    //!< Distance of target point on trajectory. (targetDistance >> tolerance)
    private ConnectedNode connectedNode;
    private Point2f actualP2F;
    private Quaternion quaternion;
    Publisher<Twist> testPublisher;
    private boolean goalAchieved = false;
    private final CountDownLatch latch = new CountDownLatch(1);


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
        if(plannedSpeed <  0) plannedSpeed =0;
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
        findTarget(actualP2F);

        if (closeEnough(actualP2F) == true && trajectory.isEmpty()) {
          //  System.out.println("GOAL ACHIEVED");
            goalAchieved = true;
        }
        if(goalAchieved)
        {
            angleCommand = 0.0;
            speedCommand = 0.0;
        }
        else
        {
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

    public void setNewPlan(LinkedBlockingQueue<WPAction> trajectory) {
        synchronized (this) {
            this.trajectory = trajectory;
            lastRemoved = point3ftoPoint2f(trajectory.peek().getPosition());
            plannedSpeed = trajectory.peek().getSpeed();
            if(plannedSpeed <  0) plannedSpeed =0;
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
    public Vector3f getVelocityVector() throws InterruptedException {
        latch.await();
       // return rotate_vector_by_quaternion(new Vector3f(1f,1f,0f),quaternion);
       // return quatToMatrix(new Vector3f(getActualP2F().x,getActualP2F().getY(),0f),quaternion);
        set(quaternion);
        return quatToMatrix(new Vector3f(1f,0f,0f),quaternion);
    }
    private Vector3f rotate_vector_by_quaternion2(Vector3f v, Quaternion q)
    {
        // Extract the vector part of the quaternion
        Vector3f u = new Vector3f((float)q.getX(), (float)q.getY(), (float)q.getZ());

        // Extract the scalar part of the quaternion
        float s = (float)q.getW();

        // Do the math

     /*   vprime = 2.0f * u.dot(v) * u
                + (s*s - dot(u, u)) * v
                + 2.0f * s * cross(u, v);
*/
        Vector3f first = new Vector3f(2.0f*u.dot(v)*u.getX(),2.0f*u.dot(v)*u.getY(),2.0f*u.dot(v)*u.getZ());
        Vector3f second = new Vector3f(s*s - u.dot(u)*v.getX(),s*s - u.dot(u)*v.getY(),s*s - u.dot(u)*v.getZ());
        Vector3f cross = new Vector3f();
        cross.cross(u,v);
        Vector3f third = new Vector3f(2.0f * s * cross.getX(),2.0f * s * cross.getY(),2.0f * s * cross.getZ());
        Vector3f result = new Vector3f();
        result.add(first,second);
        result.add(third);
        return result;
    }
    private Vector3f rotate_vector_by_quaternion(Vector3f v,Quaternion q)
    {
        float w = (float)q.getW();
        float x = (float)q.getX();
        float y = (float)q.getY();
        float z = (float)q.getZ();
        float p2x,p2y,p2z;
       float p1x = v.getX();
        float p1y = v.getY();
        float p1z = v.getZ();

        p2x = w*w*p1x + 2*y*w*p1z - 2*z*w*p1y + x*x*p1x + 2*y*x*p1y + 2*z*x*p1z - z*z*p1x - y*y*p1x;
        p2y = 2*x*y*p1x + y*y*p1y + 2*z*y*p1z + 2*w*z*p1x - z*z*p1y + w*w*p1y - 2*x*w*p1z - x*x*p1y;
        p2z = 2*x*z*p1x + 2*y*z*p1y + z*z*p1z - 2*w*y*p1x - y*y*p1z + 2*w*x*p1y - x*x*p1z + w*w*p1z;

        return new Vector3f(p2x,p2y,p2z);
        


    }
    public Vector3f quatToMatrix(Vector3f v,Quaternion q){
        double sqw = q.getW()*q.getW();
        double sqx = q.getX()*q.getX();
        double sqy = q.getY()*q.getY();
        double sqz = q.getZ()*q.getZ();

        // invs (inverse square length) is only required if quaternion is not already normalised
        double invs = 1 / (sqx + sqy + sqz + sqw);
        double m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
        double m11 = (-sqx + sqy - sqz + sqw)*invs ;
        double m22 = (-sqx - sqy + sqz + sqw) *invs ;

        double tmp1 = q.getX()*q.getY();
        double tmp2 = q.getZ()*q.getW();
        double m10 = 2.0 * (tmp1 + tmp2)*invs ;
        double m01 = 2.0 * (tmp1 - tmp2)*invs ;

        tmp1 = q.getX()*q.getZ();
        tmp2 = q.getY()*q.getW();
        double m20 = 2.0 * (tmp1 - tmp2)*invs ;
        double m02 = 2.0 * (tmp1 + tmp2)*invs ;
        tmp1 = q.getY()*q.getZ();
        tmp2 = q.getX()*q.getW();
        double m21 = 2.0 * (tmp1 + tmp2)*invs ;
        double m12 = 2.0 * (tmp1 - tmp2)*invs ;
        //------------------------------------------------
        //end of creation of the rotation matrix M
        // ------------------------------------------------

        double vyslx = m00*v.getX() + m01*v.getY() + m02*v.getZ();
        double vysly = m10*v.getX() + m11*v.getY() + m12*v.getZ();
        double vyslz = m20*v.getX() + m21*v.getY() + m22*v.getZ();
        return new Vector3f((float)vyslx,(float)vysly,(float)vyslz);
    }
    public void set(Quaternion q1) {
        double test = q1.getX()*q1.getY() + q1.getZ()*q1.getW();
        double heading,attitude,bank;
        if (test > 0.499) { // singularity at north pole
             heading = 2 * Math.atan2(q1.getX(),q1.getW());
            attitude = Math.PI/2;
             bank = 0;
            return;
        }
        if (test < -0.499) { // singularity at south pole
            heading = -2 * Math.atan2(q1.getX(), q1.getW());
            attitude = - Math.PI/2;
            bank = 0;
            return;
        }
        double sqx = q1.getX()*q1.getX();
        double sqy = q1.getY()*q1.getY();
        double sqz = q1.getZ()*q1.getZ();
        heading = Math.atan2(2 * q1.getY() * q1.getW() - 2 * q1.getX() * q1.getZ(), 1 - 2 * sqy - 2 * sqz);
        attitude = Math.asin(2 * test);
        bank = Math.atan2(2 * q1.getX() * q1.getW() - 2 * q1.getY() * q1.getZ(), 1 - 2 * sqx - 2 * sqz);
        System.out.println("Biiiiiird");
    }
}
