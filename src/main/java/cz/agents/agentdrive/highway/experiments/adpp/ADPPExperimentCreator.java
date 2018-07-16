package cz.agents.agentdrive.highway.experiments.adpp;

import cz.agents.agentdrive.highway.experiments.ExperimentCreator;

/**
 * ADPP Agent specific experiments
 * Created by wmatex on 27.11.14.
 */
public class ADPPExperimentCreator extends ExperimentCreator {

    /**
     * Initialize experiment-specific options
     */
    @Override
    protected void initExperimentOptions() {
        optionBuilder
        // Experiment-specific options
        .add("agents"      , "* Number of agents. Must be less or equal to the number of routes configured in selected map", true, true)
        .add("speeds"      , "* Number of speeds. The speed array is generated from 1 to speeds", true, true)
        .add("map"         , "* Which scenario to use", true, true)
        .add("heuristic"   , "* Which heuristic to use. Choose one of: perfect, distance", true, true)
        .add("movePenalty" , "* Penalty for moving per seconds", true, true)
        .add("waitPenalty" , "* Penalty for waiting per seconds", true, true)
        .add("waitDuration", "[1] Duration of the wait edge in time-space graph", true, false);
    }
}
