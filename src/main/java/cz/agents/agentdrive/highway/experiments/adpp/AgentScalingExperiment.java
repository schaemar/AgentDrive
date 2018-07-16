package cz.agents.agentdrive.highway.experiments.adpp;

import cz.agents.agentdrive.highway.agent.ADPPAgent;

/**
 * Experiment scaling the number of agents
 * Created by wmatex on 26.11.14.
 */
public class AgentScalingExperiment extends ADPPExperiment {
    /**
     * Run the experiment with the scaling quality
     *
     * @param quality Current value for the measured quality
     * @return
     */
    @Override
    public double run(double quality, boolean verbose) {
        // Quality is number of agents
        float[] speedArray = new float[speeds];
        for (int i = 1; i <= speeds; ++i) {
            speedArray[i-1] = i;
        }
        double exp = 0;
        for (int i = 0; i < quality; ++i) {
            ADPPAgent agent = new  ADPPAgent(i, roadnet, speedArray, waitPenalty, movePenalty, heuristic, waitDuration, false, verbose, (i==0));
            exp = Math.max(exp, agent.getExpandedNodes());
        }
        return exp;
    }
}
