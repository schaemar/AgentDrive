package cz.agents.highway.experiments.adpp;

import cz.agents.highway.environment.planning.Timer;
import cz.agents.highway.experiments.Experiment;
import org.apache.commons.cli.*;
import org.apache.log4j.LogManager;
import sun.util.logging.PlatformLogger;

import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Create and run selected number of experiments
 * Created by wmatex on 26.11.14.
 */
public class ExperimentCreator {
    private final Options commandLineOptions = new Options();
    private CommandLine commandLine;

    // Command line arguments
    String map, log, heuristic;
    double start, step, end;
    int agents, speeds, waitDuration;
    double movePenalty, waitPenalty;
    int n;

    public ExperimentCreator(String[] args) throws ParseException {
        initCommandLineOptions(commandLineOptions);
        CommandLineParser parser = new BasicParser();
        commandLine = parser.parse(commandLineOptions, args);
    }

    public static void main(String[] args) {
        try {
            // Initialize the experiments
            ExperimentCreator creator = new ExperimentCreator(args);

            // Run the experiments
            System.exit(creator.run());
        } catch (ParseException e) {
            System.err.println("Error parsing command line options: "+e);
            System.exit(1);
        }
    }

    public int run() {
        if (commandLine.hasOption("h") || !checkCommandLineArguments()) {
            HelpFormatter formatter = new HelpFormatter();
            formatter.printHelp( "ExperimentCreator", commandLineOptions);
            return 1;
        }

        Experiment experiment;
        // Choose experiment
        String type = commandLine.getOptionValue("type");
        if (type.equals("AgentScaling")) {
            experiment = new AgentScalingExperiment();
        } else if (type.equals("SpeedScaling")) {
            experiment = new SpeedScalingExperiment();
        } else {
            System.err.println("Unsupported experiment type: "+type);
            return 2;
        }

        // Parse the command line arguments to variables
        parseCommandLine(commandLine);

        // Initialize the experiment
        experiment.setUp(agents, speeds, waitDuration,
                map, heuristic,
                waitPenalty, movePenalty);

        boolean verbose = Boolean.parseBoolean(commandLine.getOptionValue("v", "false"));

        LogManager.getRootLogger().setLevel(org.apache.log4j.Level.ERROR);

        // Finally run the set of experiments
        Timer timer = new Timer(false);
        for (double quality = start; quality <= end; quality += step) {
            double sum = 0;
            for (int j = 0; j < n; ++j) {
                timer.reset();
                experiment.run(quality, verbose);
                sum += timer.getRawElapsedTime();
            }
            System.out.println("Run "+n+" experiments for quality = "+quality+", average time: "+(sum/n));
        }

        return 0;
    }

    private void parseCommandLine(CommandLine c) {
        start = Double.parseDouble(c.getOptionValue("start"));
        step  = Double.parseDouble(c.getOptionValue("step", "1"));
        end   = Double.parseDouble(c.getOptionValue("end"));
        movePenalty   = Double.parseDouble(c.getOptionValue("movePenalty"));
        waitPenalty   = Double.parseDouble(c.getOptionValue("waitPenalty"));

        map  = c.getOptionValue("map");
        heuristic = c.getOptionValue("heuristic");
        log = c.getOptionValue("log");

        n = Integer.parseInt(c.getOptionValue("n", "1"));
        speeds = Integer.parseInt(c.getOptionValue("speeds"));
        agents = Integer.parseInt(c.getOptionValue("agents"));
        waitDuration = Integer.parseInt(c.getOptionValue("waitDuration", "1"));
    }
    /**
     * Initialize command line options
     * @param opts
     */
    private void initCommandLineOptions(Options opts) {
        // Boolean options
        opts.addOption("h", false, "Show help");
        opts.addOption("v", false, "Show more verbose output");

        // Options with value
        opts.addOption("log"  , true, "Save output to given file");
        opts.addOption("n"    , true, "[1] Run each experiment n-times and produce mean measurements");
        opts.addOption("start", true, "* Start value for the measured quality");
        opts.addOption("step" , true, "[1] Step for increasing the measured quality");
        opts.addOption("end"  , true, "* End value for the measured quality");
        opts.addOption("type" , true, "* Type of the experiment. Available: AgentScaling, SpeedScaling, GraphScaling");

        // Experiment-specific options
        opts.addOption("agents"      , true, "* Number of agents. Must be less or equal to the number of routes configured in selected map");
        opts.addOption("speeds"      , true, "* Number of speeds. The speed array is generated from 1 to speeds");
        opts.addOption("map"         , true, "* Which scenario to use");
        opts.addOption("heuristic"   , true, "* Which heuristic to use. Choose one of: perfect, distance");
        opts.addOption("movePenalty" , true, "* Penalty for moving per seconds");
        opts.addOption("waitPenalty" , true, "* Penalty for waiting per seconds");
        opts.addOption("waitDuration", true, "[1] Duration of the wait edge in time-space graph");
    }

    /**
     * Check whether all required options are passed.
     * @return
     */
    private boolean checkCommandLineArguments() {
        return commandLine.hasOption("start") &&
                commandLine.hasOption("end") &&
                commandLine.hasOption("type") &&
                commandLine.hasOption("agents") &&
                commandLine.hasOption("speeds") &&
                commandLine.hasOption("map") &&
                commandLine.hasOption("heuristic") &&
                commandLine.hasOption("movePenalty") &&
                commandLine.hasOption("waitPenalty");
    }
}

