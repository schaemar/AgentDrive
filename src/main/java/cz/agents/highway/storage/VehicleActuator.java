package cz.agents.highway.storage;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.environment.eventbased.EventBasedAction;
import cz.agents.alite.environment.eventbased.EventBasedEnvironment;
import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.util.Utils;

import java.util.LinkedList;
import java.util.List;

public class VehicleActuator extends EventBasedAction {

    private HighwayStorage storage;
    private int id;

    public VehicleActuator(EventBasedEnvironment environment, Entity relatedEntity, HighwayStorage storage) {
        super(environment, relatedEntity);
        this.storage = storage;
        this.id = Utils.name2ID(relatedEntity.getName());
    }
    
    public void act(Action action){
        List<Action> actions = new LinkedList<Action>();
        actions.add(action);
        act(actions);
    }
    public void act(List<Action> actions){
        storage.act(id, actions);
        getEventProcessor().addEvent(HighwayEventType.NEW_PLAN, null, null, actions);
    }
}
