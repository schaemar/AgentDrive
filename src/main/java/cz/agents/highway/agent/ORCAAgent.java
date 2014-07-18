package cz.agents.highway.agent;

import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;
import org.apache.log4j.Logger;
import rvolib.MutableFloat;
import rvolib.RVOAgent;
import rvolib.Vector2;

import javax.vecmath.Point3f;

/**
 * Created by martin on 14.7.14.
 */
public class ORCAAgent extends RouteAgent {
    Logger log = Logger.getLogger(RouteAgent.class);

    float TIMESTEP = .1f;

    RVOAgent rvoAgent;



    public ORCAAgent(int id) {
        super(id);
        this.rvoAgent = new RVOAgent(id, new Vector2(this.getInitialPosition().x, this.getInitialPosition().y));
        rvoAgent.initVisualization();


    }

    @Override
    protected Action agentReact() {
        RoadObject me = sensor.senseCurrentState();

        // Simulator did not send update yet
        if (me == null) {
            return new WPAction(id, 0d, getInitialPosition(), 0);
        }

        log.debug("Timestep = "+ TIMESTEP);
        Vector2 velocity = ORCAUtil.vector3fToVector2(me.getVelocity());
        Vector2 position = ORCAUtil.vector3fToVector2(me.getPosition());
        rvoAgent.position_ = position;
        rvoAgent.velocity_ = velocity;


        WPAction desiredAction = (WPAction) super.agentReact();
        Vector2 prefVel = new Vector2(desiredAction.getPosition().x - position.x(), desiredAction.getPosition().y - position.y());

        //scale preferred velocity to correspond desired TIMESTEP to simulate
        prefVel = prefVel.scale((float) (1 / prefVel.getLength()), prefVel);
        prefVel =  prefVel.scale((float) desiredAction.getSpeed() , prefVel);
        prefVel = prefVel.scale(TIMESTEP, prefVel);

       log.debug("prefVel = "+prefVel.getLength());

        rvoAgent.setPrefVelocity(prefVel);

        // compute Neighbors - agents and obstacles
        rvoAgent.clearAgentNeighbor();
        for (RoadObject roadObject : sensor.senseCars()) {
            if (roadObject.getId() != me.getId()) {
                RVOAgent other = new RVOAgent(0, ORCAUtil.vector3fToVector2(roadObject.getPosition()));
                other.velocity_ = ORCAUtil.vector3fToVector2(roadObject.getVelocity());
//                if(me.getId()==0)other.initVisualization();
                rvoAgent.insertAgentNeighbor(other, new MutableFloat(1000000));
            }
        }

        //rvoAgent.insertObstacleNeighbor(obstacle, rangeSq);

        Vector2 newVelocity = rvoAgent.computeNewVelocity(TIMESTEP);
        double speed = newVelocity.getLength()/ TIMESTEP;
        log.debug("ORCA out speed= "+speed);
        Point3f p = new Point3f(me.getPosition());
        p.add(ORCAUtil.vector2ToVector3f(newVelocity));
        WPAction action = new WPAction(id, me.getUpdateTime(), p, speed);

        return action;
    }


}
