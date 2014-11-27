package cz.agents.highway.experiments;

import cz.agents.highway.environment.roadnet.XMLReader;

/**
 * General interface for various scaling experiments
 * Created by wmatex on 26.11.14.
 */
public abstract class Experiment {
    protected int agents, speeds, waitDuration;
    protected String heuristic;
    protected double waitPenalty, movePenalty;
    /**
     * Run the experiment with the scaling quality
     *
     * @param quality Current value for the measured quality
     * @return
     */
    public abstract boolean run(double quality, boolean verbose);

    /**
     * Initialize the experiment
     */
    protected abstract void init();

    public void setUp(int agents, int speeds, int waitDuration, String map, String heuristic,
                      double waitPenalty, double movePenalty) {
        this.agents = agents;
        this.speeds = speeds;
        this.waitDuration = waitDuration;
        this.heuristic = heuristic;
        this.waitPenalty = waitPenalty;
        this.movePenalty = movePenalty;

        createMap(map);
        init();
    }

    private void createMap(String map) {
        XMLReader.getInstance().read(map);
    }
}
