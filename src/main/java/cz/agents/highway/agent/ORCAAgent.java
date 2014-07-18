package cz.agents.highway.agent;

import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.element.Point;
import cz.agents.alite.vis.element.aggregation.PointElements;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.terminal.PointLayer;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;
import rvolib.*;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;
import java.awt.*;
import java.util.ArrayList;

/**
 * Created by martin on 14.7.14.
 */
public class ORCAAgent extends RouteAgent {
    RVOAgent rvoAgent;


    public ORCAAgent(int id) {
        super(id);
        this.rvoAgent = new RVOAgent(id,new Vector2(this.getInitialPosition().x,this.getInitialPosition().y));
        rvoAgent.initVisualization();


    }

    @Override
    protected Action agentReact() {
        RoadObject me = sensor.senseCurrentState();

        // Simulator did not send update yet
        if (me == null) {
            return new WPAction(id, 0d, getInitialPosition(), 0);
        }
        float timestep = 0.1f;
        Vector2 velocity = ORCAUtil.vector3fToVector2(me.getPosition());
        Vector2 position = ORCAUtil.vector3fToVector2(me.getPosition());
        rvoAgent.position_ = position;
        rvoAgent.velocity_ = velocity;


        WPAction desiredAction = (WPAction) super.agentReact();
        Vector2 prefVel = new Vector2(desiredAction.getPosition().x - position.x(), desiredAction.getPosition().y - position.y());

        //prefVel.scale((float)1,prefVel);
        //prefVel.scale((float) desiredAction.getSpeed(), prefVel);
        rvoAgent.setPrefVelocity(prefVel);
        // compute Neighbors - agents and obstacles
        rvoAgent.clearAgentNeighbor();
        for (RoadObject roadObject : sensor.senseCars()) {
            if(roadObject.getId()!=me.getId()) {
                RVOAgent other = new RVOAgent(0, ORCAUtil.vector3fToVector2(roadObject.getPosition()));
                other.velocity_ = ORCAUtil.vector3fToVector2(roadObject.getVelocity());
//                if(me.getId()==0)other.initVisualization();
                rvoAgent.insertAgentNeighbor(other, new MutableFloat(me.getPosition().distanceSquared(roadObject.getPosition())));
            }
        }

        //rvoAgent.insertObstacleNeighbor(obstacle, rangeSq);


        Vector2 newVelocity = rvoAgent.computeNewVelocity(timestep);
        Point3f p = new Point3f(me.getPosition());
        p.add(ORCAUtil.vector2ToVector3f(newVelocity));
        WPAction action = new WPAction(id, me.getUpdateTime(), p, newVelocity.getLength());

        //rvoAgent.update(timestep,velocity);
        return action;
    }


}
