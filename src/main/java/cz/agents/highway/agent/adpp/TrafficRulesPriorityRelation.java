package cz.agents.highway.agent.adpp;

import cz.agents.highway.agent.adpp.PriorityRelation;
import cz.agents.highway.environment.planning.euclid4d.Region;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.VehicleSensor;

import java.util.*;

/**
 * Set the priorities as defined by the traffic rules
 * Created by wmatex on 15.3.15.
 */
public class TrafficRulesPriorityRelation implements PriorityRelation {
    private final VehicleSensor sensor;
    private final int agentId;

    public TrafficRulesPriorityRelation(final VehicleSensor sensor) {
        this.sensor = sensor;
        this.agentId = sensor.getId();
    }

    /**
     * Return id's of higher priority agents
     *
     * @return
     */
    @Override
    public List<Integer> higherPriority() {
        List<Integer> higherPriority = new LinkedList<Integer>();
        for (RoadObject agent: sensor.senseCars()) {
            if (agent.getId() > agentId) {
                higherPriority.add(agent.getId());
            }
        }
        return higherPriority;
    }

    /**
     * Return id's of lower priority agents
     */
    @Override
    public List<Integer> lowerPriority() {
        List<Integer> lowerPriority = new LinkedList<Integer>();
        for (RoadObject agent: sensor.senseCars()) {
            if (agent.getId() < agentId) {
                lowerPriority.add(agent.getId());
            }
        }
        return lowerPriority;
    }
}
