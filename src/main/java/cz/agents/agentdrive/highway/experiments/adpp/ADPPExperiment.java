package cz.agents.agentdrive.highway.experiments.adpp;

import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.experiments.Experiment;
import cz.agents.agentdrive.highway.experiments.FileLogger;
import org.apache.commons.cli.CommandLine;
import org.apache.commons.cli.ParseException;
/**
 * General ADPPExperiment
 * Created by wmatex on 27.11.14.
 */
public abstract class ADPPExperiment implements Experiment {
    protected int agents, speeds, waitDuration;
    protected String heuristic, map;
    protected double waitPenalty, movePenalty;
    protected RoadNetwork roadnet;

    /**
     * Initialize the experiment
     */
    public void setUp(CommandLine c) throws ParseException {
        agents = Integer.parseInt(c.getOptionValue("agents"));
        speeds = Integer.parseInt(c.getOptionValue("speeds"));
        waitDuration = Integer.parseInt(c.getOptionValue("waitDuration", "1"));

        heuristic = c.getOptionValue("heuristic");
        map = c.getOptionValue("map");

        waitPenalty = Double.parseDouble(c.getOptionValue("waitPenalty"));
        movePenalty = Double.parseDouble(c.getOptionValue("movePenalty"));

        createMap(map);
    }

    private void createMap(String map) {
        XMLReader reader = new XMLReader();
        roadnet = reader.parseNetwork(map);
    }

    @Override
    public void log(FileLogger logger) {
        logger
                .addHeaderValue("Agents", agents)
                .addHeaderValue("Speeds", speeds)
                .addHeaderValue("WaitDuration", waitDuration)
                .addHeaderValue("Heuristic", heuristic)
                .addHeaderValue("Map", map)
                .addHeaderValue("WaitPenalty", waitPenalty)
                .addHeaderValue("MovePenalty", movePenalty);
    }
}
