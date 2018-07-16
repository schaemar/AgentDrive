package cz.agents.agentdrive.highway.agent.adpp;

import java.util.List;

/**
 * Determine the priority relations between the agents
 * Created by wmatex on 15.3.15.
 */
public interface PriorityRelation {
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
