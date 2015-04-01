package cz.agents.highway.agent;

import cz.agents.alite.environment.Sensor;
import cz.agents.highway.storage.HighwayEventType;
import cz.agents.highway.storage.HighwayStorage;
import cz.agents.highway.storage.VehicleSensor;
import tt.euclidtime3i.Region;

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
     * Return the trajectories of higher-priority agents represented as moving obstacles
     *
     * @return
     */
    @Override
    public List<Region> movingObstacles() {
        List<Region> obstacles = new LinkedList<Region>();
        Map<Integer, Region> trajectories = sensor.senseTrajectories();
        for (int id: trajectories.keySet()) {
            if (id > agentId) {
                obstacles.add(trajectories.get(id));
            }
        }
        return obstacles;
    }

    /**
     * Return id's of higher priority agents
     *
     * @return
     */
    @Override
    public List<Integer> higherPriority() {
        List<Integer> higherPriority = new LinkedList<Integer>();
        Map<Integer, Region> trajectories = sensor.senseTrajectories();
        for (int id: trajectories.keySet()) {
            if (id > agentId) {
//                if (agentId == 20 && id == 21) continue;
//                if (agentId == 22 && id == 23) continue;
//                if (agentId == 24 && id == 25) continue;

                higherPriority.add(id);
            }
        }
//        if (agentId == 21 && trajectories.containsKey(20)) {
//            higherPriority.add(20);
//        } else  if (agentId == 23 && trajectories.containsKey(22)) {
//            higherPriority.add(22);
//        } else if (agentId == 25 && trajectories.containsKey(24)) {
//            higherPriority.add(24);
//        }
        return higherPriority;
    }

    /**
     * Return id's of lower priority agents
     */
    @Override
    public List<Integer> lowerPriority() {
        return null;
    }
}
