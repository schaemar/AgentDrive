package cz.agents.highway.creator;

import cz.agents.alite.configurator.Configurator;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.PlansOut;
import cz.agents.highway.storage.plan.WPAction;
import geometry_msgs.Twist;
import geometry_msgs.Vector3;
import org.ros.concurrent.CancellableLoop;
import org.ros.message.MessageFactory;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import javax.vecmath.*;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.LinkedBlockingQueue;

/**
 * Created by david on 7/14/15.
 */
public class RosControlNode extends AbstractNodeMain implements NodeMain {
    ConnectedNode connectedNode = null;
    private long nodeStartedAt;
    Publisher<Twist> testPublisher;
    TrajectoryFollowing trajectoryFollowing;
    private final static float TOLERANCE = 0.05f;
    private final static float SPEED = 0.15f;
    private final static float LOOKAHEAD = 0.2f;
    private Map<Integer,TrajectoryFollowing> mapTraj = new LinkedHashMap<Integer, TrajectoryFollowing>();
    private Map<String,TrajectoryFollowing> chanelMap = new LinkedHashMap<String, TrajectoryFollowing>();
    private final CountDownLatch latch = new CountDownLatch(1);
    private final static float SCALECONSTANT = Configurator.getParamDouble("highway.dashboard.rosconfig.scale", 1.0).floatValue();;


    public RosControlNode() {
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("AgentDriveNode");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {

        this.connectedNode = connectedNode;
        // sleep to get clock
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
        }
//        nodeStartedAt = getRosTimeSec() * 1000l;

        //  setupPublishing(connectedNode);
        //  setupSubscriptions(connectedNode);

        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override
            protected void setup() {
            }

            @Override
            protected void loop() throws InterruptedException {
                spin();
            }
        });
        latch.countDown();
    }

    @Override
    public void onShutdown(Node node) {
        MessageFactory msgFactory = connectedNode.getTopicMessageFactory();
        Twist twist = msgFactory
                .newFromType(Twist._TYPE);
        Vector3 vector = msgFactory
                .newFromType(Vector3._TYPE);

        vector.setX(0);
        vector.setY(0);
        vector.setZ(0);

        twist.setLinear(vector);

        // testPublisher.publish(twist);

        super.onShutdown(node);
    }


    private void spin() {
        // publishTwist();
        // System.out.println("aaaa");
    }

    private long getRosTimeSec() {
        boolean simTime = false;
        if (connectedNode.getParameterTree().has("/use_sim_time")) {
            simTime = connectedNode.getParameterTree().getBoolean(
                    "/use_sim_time");
        }
        if (simTime) {
            return connectedNode.getCurrentTime().secs;
        } else {
            return (System.currentTimeMillis() / 1000l);
        }
    }

    private Publisher<Twist> setupPublishing(ConnectedNode connectedNode, String robotString) {
        return connectedNode.newPublisher(robotString + "/cmd_vel",
                Twist._TYPE);
        //   trajectoryFollowing = new TrajectoryFollowing(connectedNode,testPublisher,trajectory,TOLERANCE,SPEED,LOOKAHEAD);

    }

    private void setupSubscriptions(ConnectedNode connectedNode, final String robotString) {

        Subscriber<nav_msgs.Odometry> trajInfo = connectedNode.newSubscriber(
                robotString + "/odom", nav_msgs.Odometry._TYPE);
        // Subscriber<nav_msgs.Odometry> trajInfo = connectedNode.newSubscriber(
        //         "/base_pose_ground_truth", nav_msgs.Odometry._TYPE);
        trajInfo.addMessageListener(new MessageListener<nav_msgs.Odometry>() {

            public void onNewMessage(nav_msgs.Odometry od) {
                    chanelMap.get(robotString).messageCallback(od);
                //              processOdometry(od);
            }
        });

    }

    public void executePlans(PlansOut plans) throws InterruptedException{
      //  while (connectedNode == null){} //TODO FIX THIS!!!!!!
        latch.await();
        for (Integer carID : plans.getCarIds()) {
            Collection<Action> plan = plans.getPlan(carID);
            LinkedBlockingQueue<WPAction> forNode = new LinkedBlockingQueue<WPAction>(plan.size());
            for (Action action : plan) {
                if (action.getClass().equals(WPAction.class)) {
                    WPAction wpAction = (WPAction) action;
                    try {
                        WPAction converted = new WPAction(wpAction.getCarId(),wpAction.getTimeStamp(),
                                new Point3f(Math.abs(wpAction.getPosition().getX()/SCALECONSTANT),Math.abs(wpAction.getPosition().getY()/SCALECONSTANT),0),wpAction.getSpeed()/SCALECONSTANT);
                        forNode.put(converted);
                    }
                    catch (InterruptedException e)
                    {
                        System.out.println(e.getMessage());
                    }
                }
            }
            if(mapTraj.containsKey(carID))
            {
                mapTraj.get(carID).setNewPlan(forNode);
            }
            else
            {
                String robotString = "robot_" + carID;

                Publisher<Twist> pubForNode = setupPublishing(connectedNode,robotString);
                setupSubscriptions(connectedNode,robotString);
                TrajectoryFollowing newTraj = new TrajectoryFollowing(connectedNode,pubForNode,forNode,TOLERANCE,SPEED,LOOKAHEAD);
                chanelMap.put(robotString,newTraj);
                mapTraj.put(carID,newTraj);
            }
        }
    }
    public RadarData generateRadarData()
    {
        RadarData radarData = new RadarData();
        for (Map.Entry<Integer, TrajectoryFollowing> obje : mapTraj.entrySet()) {
           // int lane = highwayEnvironment.getRoadNetwork().getLaneNum(myPosition);
            //while (obje.getValue().getActualP2F() == null){} //TODO FIX THIS
            int count = 0;
            int maxTries = 3;
            while(true) {
                try {
                    Point2f target = obje.getValue().getTarget();
                    Point2f actual = obje.getValue().getActualP2F();

                      Vector3f vel = new Vector3f(((target.getX() - actual.getX()) ), ((target.getY() - actual.getY()) *-1), 0);
                   // Vector3f vel = obje.getValue().getVelocityVector();
                //    vel.scale(10f);
                //    vel = new Vector3f(1f,0f,0f);
                    vel.normalize();
                    vel.scale((float)obje.getValue().getActualSpeed()*SCALECONSTANT);
                    RoadObject state = new RoadObject(obje.getKey(), System.currentTimeMillis(), 0, new Point3f(actual.getX() * SCALECONSTANT, -actual.getY() * SCALECONSTANT, 0), vel);
                    radarData.add(state);
                    break;
                } catch (InterruptedException e) {
                    if (++count == maxTries)
                    {
                        System.out.println(e.getMessage());
                    }
                }
            }
        }
        return radarData;
    }
}
