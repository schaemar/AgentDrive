package cz.agents.highway.storage;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.environment.eventbased.EventBasedAction;
import cz.agents.alite.environment.eventbased.EventBasedEnvironment;
import cz.agents.highway.storage.plan.Action;
import cz.agents.highway.util.Utils;

public class VehicleActuator extends EventBasedAction {

    private HighwayStorage storage;
    private int id;

    public VehicleActuator(EventBasedEnvironment environment, Entity relatedEntity, HighwayStorage storage) {
        super(environment, relatedEntity);
        this.storage = storage;
        this.id = Utils.name2ID(relatedEntity.getName());
    }
    
    public void act(Action action){
        storage.act(id, action);
        getEventProcessor().addEvent(HighwayEventType.NEW_PLAN, null, null, action);
    }

}
