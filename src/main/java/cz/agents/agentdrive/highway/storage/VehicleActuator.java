package cz.agents.agentdrive.highway.storage;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.environment.eventbased.EventBasedAction;
import cz.agents.agentdrive.highway.environment.HighwayEnvironment;
import cz.agents.agentdrive.highway.storage.plan.Action;
import cz.agents.agentdrive.highway.util.Utils;

import java.util.LinkedList;
import java.util.List;

public class VehicleActuator {

    private HighwayStorage storage;
    private int id;
    private Entity entity;
    private HighwayEnvironment highwayEnvironment;

    public VehicleActuator(HighwayEnvironment environment, Entity relatedEntity, HighwayStorage storage) {
        this.highwayEnvironment = environment;
        this.entity = relatedEntity;
        this.storage = storage;
        this.id = Utils.name2ID(relatedEntity.getName());
    }

    public void act(Action action) {
        List<Action> actions = new LinkedList<Action>();
        actions.add(action);
        act(actions);
    }

    public List<Action> act(List<Action> actions) {
        storage.act(id, actions);
        return actions;
        //getEventProcessor().addEvent(HighwayEventType.NEW_PLAN, null, null, actions);
    }
}
