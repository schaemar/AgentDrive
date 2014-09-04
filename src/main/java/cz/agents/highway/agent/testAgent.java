package cz.agents.highway.agent;

import cz.agents.alite.common.event.Event;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.storage.plan.WPAction;

import javax.vecmath.Point3f;
import java.util.List;

/**
 * Created by david on 26.8.14.
 */
public class testAgent extends Agent {
    public testAgent(int id) {
        super(id);
    }
    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        this.sensor.registerReaction(new Reaction() {
            public void react(Event event) {
                if (event.getType().equals(HighwayEventType.UPDATED)) {
                    actuator.act(agentReact());
                }
            }
        });
    }
        protected Action agentReact () {
            RoadObject me = sensor.senseCurrentState();
            if (me == null) {
                return new WPAction(id, 0d, getInitialPosition(), 0);
            }
            return new WPAction(sensor.getId(), me.getUpdateTime(),
                    new Point3f(0, 0, me.getPosition().z), 80);
        }
    }
