package cz.agents.highway.agent;

import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.environment.roadnet.Network;
import cz.agents.highway.protobuf.generated.simplan.VectorProto;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;
import org.apache.log4j.Logger;
import rvolib.*;
import tt.euclid2i.Point;
import tt.euclid2i.region.Polygon;
import tt.euclid2i.util.Util;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import javax.vecmath.Vector2f;
import java.util.ArrayList;

/**
 * Created by martin on 14.7.14.
 */
public class ORCAAgent extends RouteAgent {
    Logger log = Logger.getLogger(RouteAgent.class);

    float TIMESTEP = .1f;
    //TODO what is rangeSQ for
    MutableFloat rangeSq = new MutableFloat(1000000);

    RVOAgent rvoAgent;
    ArrayList<RVOObstacle> obstacles = null;


    public ORCAAgent(int id) {
        super(id);
        this.rvoAgent = new RVOAgent(id, new Vector2(this.getInitialPosition().x, this.getInitialPosition().y));
        rvoAgent.initVisualization();
    }

    private ArrayList<RVOObstacle> generateObstacles(Network roadNetwork) {
        float shift = 5;
        ArrayList<RVOObstacle> obstacles = new ArrayList<RVOObstacle>();
        //obstacles
        for (Lane lane : roadNetwork.getLanes().values()) {

            //generate obstacles only for the rightmost lanes
            if(lane.getLaneRight()==null) {

                ArrayList<Point2f> points = lane.getShape();
                Point2f p1, p2;
                for (int i = 0; i < points.size(); i++) {
                    //get two neighboring points
                    p1 = points.get(i);
                    if (i + 1 < points.size()) {
                        p2 = points.get(i + 1);
                    } else {
                        p2 = points.get(0);
                    }


                    //directional vector
                    Vector2f v = new Vector2f(p1);
                    v.sub(p2);
                    v.normalize();

                    //normal vector
                    Vector2f n = new Vector2f(-v.y, v.x);
                    n.normalize();

                    ArrayList<Point> polyPoints = new ArrayList<Point>();

                    //vector to add
                    Vector2f vAdd = new Vector2f(n);
                    Point2f pNew = new Point2f(p2);
                    vAdd.scale(shift);
                    pNew.add(vAdd);
                    Point p = new Point((int) pNew.x, (int) pNew.y);
                    polyPoints.add(p);

                    vAdd = new Vector2f(n);
                    pNew = new Point2f(p1);
                    vAdd.scale(shift);
                    pNew.add(vAdd);
                    p = new Point((int) pNew.x, (int) pNew.y);
                    polyPoints.add(p);

                    vAdd = new Vector2f(n);
                    pNew = new Point2f(p1);
                    vAdd.scale(2 * shift);
                    pNew.add(vAdd);
                    p = new Point((int) pNew.x, (int) pNew.y);
                    polyPoints.add(p);

                    vAdd = new Vector2f(n);
                    pNew = new Point2f(p2);
                    vAdd.scale(2 * shift);
                    pNew.add(vAdd);
                    p = new Point((int) pNew.x, (int) pNew.y);
                    polyPoints.add(p);


//        Point[] points = new Point[4];
//        points[0] = new Point(-20, 0);
//        points[1] = new Point(-20, -600);
//        points[2] = new Point(-4, -600);
//        points[3] = new Point(-4, 0);
                    Point[] polyPointsAr = new Point[polyPoints.size()];
                    Polygon polygon = new Polygon(polyPoints.toArray(polyPointsAr));
                    ArrayList<Polygon> polygons = new ArrayList<Polygon>();
                    polygons.add(polygon);
                    ArrayList<Vector2> vertices = RVOUtil.regionToVectorList(polygon);
                    RVOUtil.addObstacle(vertices, obstacles);
                }
            }
        }
//        points = new Point[4];
//        points[0] = new Point(8, 0);
//        points[1] = new Point(8, -600);
//        points[2] = new Point(20, -600);
//        points[3] = new Point(20, 0);
//        Polygon polygon2 = new Polygon(points);
//        ArrayList<Vector2> vertices2 = RVOUtil.regionToVectorList(polygon2);
//        RVOUtil.addObstacle(vertices2, obstacles);

        return obstacles;
    }

    @Override
    protected Action agentReact() {
        RoadObject me = sensor.senseCurrentState();


        // Simulator did not send update yet
        if (me == null) {
            return new WPAction(id, 0d, getInitialPosition(), 0);
        }

//generate obstacles for ORCA algorithm according to roadNetwork
        if(obstacles ==null)obstacles = generateObstacles(sensor.getRoadDescription().getRoadNetwork());


        //insert obstacles to agent's world representation
        for (RVOObstacle rvoObstacle : obstacles) {
            rvoAgent.insertObstacleNeighbor(rvoObstacle, rangeSq.getValue());
        }

        log.debug("Timestep = " + TIMESTEP);
        Vector2 velocity = ORCAUtil.vector3fToVector2(me.getVelocity());
        Vector2 position = ORCAUtil.vector3fToVector2(me.getPosition());
        rvoAgent.position_ = position;
        rvoAgent.velocity_ = velocity;


        WPAction desiredAction = (WPAction) super.agentReact();
        Vector2 prefVel = new Vector2(desiredAction.getPosition().x - position.x(), desiredAction.getPosition().y - position.y());

        //scale preferred velocity to correspond desired TIMESTEP to simulate
        prefVel = prefVel.scale((float) (1 / prefVel.getLength()), prefVel);
        prefVel = prefVel.scale((float) desiredAction.getSpeed(), prefVel);
        prefVel = prefVel.scale(TIMESTEP, prefVel);

        log.debug("prefVel = " + prefVel.getLength());

        rvoAgent.setPrefVelocity(prefVel);


        // compute Neighbors - agents and obstacles
        rvoAgent.clearAgentNeighbor();
        for (RoadObject roadObject : sensor.senseCars()) {
            if (roadObject.getId() != me.getId()) {
                RVOAgent other = new RVOAgent(0, ORCAUtil.vector3fToVector2(roadObject.getPosition()));
                other.velocity_ = ORCAUtil.vector3fToVector2(roadObject.getVelocity());
//                if(me.getId()==0)other.initVisualization();
                rvoAgent.insertAgentNeighbor(other, rangeSq);
            }
        }


        Vector2 newVelocity = rvoAgent.computeNewVelocity(TIMESTEP);
        double speed = newVelocity.getLength();
        log.debug("ORCA out speed= " + speed);
        Point3f p = new Point3f(me.getPosition());
        p.add(ORCAUtil.vector2ToVector3f(newVelocity));
        WPAction action = new WPAction(id, me.getUpdateTime(), p, speed);

        return action;
    }


}
