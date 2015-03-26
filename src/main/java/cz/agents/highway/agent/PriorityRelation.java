package cz.agents.highway.agent;

import tt.euclidtime3i.Region;

import java.util.List;

/**
 * Determine the priority relations between the agents
 * Created by wmatex on 15.3.15.
 */
public interface PriorityRelation {
    /**
     * Return the trajectories of higher-priority agents represented as moving obstacles
     * @return
     */
    List<Region> movingObstacles();

    /**
     * Return id's of higher priority agents
     * @return
     */
    List<Integer> higherPriority();

    /**
     * Return id's of lower priority agents
     */
    List<Integer> lowerPriority();
}
