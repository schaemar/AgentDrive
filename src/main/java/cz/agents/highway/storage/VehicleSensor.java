package cz.agents.highway.storage;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import cz.agents.alite.common.entity.Entity;
import cz.agents.alite.common.event.Event;
import cz.agents.alite.environment.eventbased.EventBasedEnvironment;
import cz.agents.alite.environment.eventbased.EventBasedSensor;
import cz.agents.highway.agent.Reaction;
import cz.agents.highway.util.Utils;
import tt.euclidtime3i.Region;

public class VehicleSensor extends EventBasedSensor {

    private HighwayStorage storage;

    public VehicleSensor(EventBasedEnvironment environment, Entity relatedEntity,
            HighwayStorage storage) {
        super(environment, relatedEntity);
        this.storage = storage;
        getEventProcessor().addEventHandler(this);
        id = Utils.name2ID(relatedEntity.getName());
    }

    private Reaction reaction;
    private int id;

    @Override
    public void handleEvent(Event event) {
        if (reaction != null) {
            reaction.react(event);
        }
       
    }

    public int getId() {
        return id;
    }

    public RoadObject senseCurrentState() {
        return storage.getPosCurr().get(id);
    }

    public Collection<RoadObject> senseCars() {
        // add all cars
        Collection<RoadObject> ret = storage.getPosCurr().values();
        return ret;
    }
    public RoadObject senseCar(int id){
        return storage.getPosCurr().get(id);
    }

    public Collection<RoadObject> senseObstacles() {
        // add all tapers
        Collection<RoadObject> ret = new ArrayList<RoadObject>(storage.getRoadDescription().getObstacles());

        return ret;
    }

    public int senseMaxLane() {
//        RoadObject state = senseCurrentState();
//        Point2d currPoint = new Point2d(state.getPosition().x,state.getPosition().y);
        return 2;//(int) storage.getRoadDescription().getNearestHighwayPoint(currPoint).z - 1;
    }

    public void registerReaction(Reaction reaction) {
        this.reaction = reaction;
    }

    public RoadDescription getRoadDescription() {
        return storage.getRoadDescription();
    }

    public Map<Integer, Region> senseTrajectories() {
        return storage.getTrajectories();
    }

}
