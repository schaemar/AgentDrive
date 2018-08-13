package cz.agents.agentdrive.highway.storage;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;

import cz.agents.alite.common.entity.Entity;
import cz.agents.agentdrive.highway.agent.Agent;
import cz.agents.agentdrive.highway.environment.HighwayEnvironment;
import cz.agents.agentdrive.highway.environment.planning.euclid4d.Region;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.util.Utils;
import org.apache.log4j.Logger;

public class VehicleSensor {

    private static final Logger logger = Logger.getLogger(VehicleSensor.class);
    private HighwayStorage storage;
    private Entity entity;
    private HighwayEnvironment highwayEnvironment;

    public VehicleSensor(HighwayEnvironment environment, Entity relatedEntity,
                         HighwayStorage storage) {
        this.highwayEnvironment = environment;
        this.entity = relatedEntity;
        this.storage = storage;
        id = Utils.name2ID(relatedEntity.getName());
    }

    private int id;


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

    public RoadObject senseCar(int id) {
        return storage.getPosCurr().get(id);
    }

    public Collection<RoadObject> senseObstacles() {
        // add all tapers
        Collection<RoadObject> ret = new ArrayList<RoadObject>(storage.getRoadDescription().getObstacles());

        return ret;
    }

    public RoadDescription getRoadDescription() {
        return storage.getRoadDescription();
    }

    public Map<Integer, Region> senseTrajectories() {
        return storage.getTrajectories();
    }

    public Map<Integer, Agent> getAgents() {
        return storage.getAgents();
    }

    public RoadNetwork getRoadNetwork() {
        return highwayEnvironment.getRoadNetwork();
    }

    public long getCurrentTime() {
        return highwayEnvironment.getCurrentTime();
    }

    public String getRelatedEntityName() {
        return entity.getName();
    }
}
