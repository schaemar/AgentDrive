package cz.agents.highway.experiments;

import cz.agents.highway.environment.planning.Timer;
import cz.agents.highway.experiments.adpp.AgentScalingExperiment;
import cz.agents.highway.experiments.adpp.SpeedScalingExperiment;
import org.apache.commons.cli.*;
import org.apache.log4j.LogManager;

/**
 * Create and run selected number of experiments
 * Created by wmatex on 26.11.14.
 */
public abstract class  ExperimentCreator {
    private final Options commandLineOptions = new Options();
    private CommandLine commandLine;
    protected final OptionBuilder optionBuilder = new OptionBuilder(commandLineOptions);

    // Command line arguments
    String log;
    double start, step, end;
    int n;
    boolean verbose;

    public ExperimentCreator() {
    }

    public void init(String[] args) throws ParseException {
        initCommandLineOptions();
        CommandLineParser parser = new BasicParser();
        try {
            commandLine = parser.parse(commandLineOptions, args);
        } catch (ParseException e) {
            System.err.println("Error parsing command line options: " + e);
            printHelp();
            throw e;
        }
    }

    public static void main(String[] args) {
        ExperimentCreator creator = null;
        try {
            // Initialize the experiments
            creator = (ExperimentCreator) Class.forName(args[0]).newInstance();
            creator.init(args);
        } catch (ParseException e1) {
            System.exit(2);
        } catch (Exception e) {
            System.exit(1);
        }

        // Run the experiments
        System.exit(creator.run());
    }

    public int run() {
        if (commandLine.hasOption("h")) {
            printHelp();
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

        try {
            // Parse the command line arguments to variables
            parseCommandLine(commandLine);

            // Initialize the experiment
            experiment.setUp(commandLine);
        } catch (ParseException e) {
            System.err.println("Error parsing command line options: "+e);
            printHelp();
            return 1;
        }

        LogManager.getRootLogger().setLevel(org.apache.log4j.Level.ERROR);

        FileLogger logger = new FileLogger(type);

        // Log useful information about experiment
        experiment.log(logger);

        // Finally run the set of experiments
        Timer timer = new Timer(false);
        for (double quality = start; quality <= end; quality += step) {
            double sum = 0, average, res = 0, res_avg;
            for (int j = 0; j < n; ++j) {
                timer.reset();
                res += experiment.run(quality, verbose);
                sum += timer.getRawElapsedTime();
            }
            average = sum/n;
            res_avg = res/n;
            logger.addPoint(quality, average);
            logger.addAltPoint(res_avg);
            System.out.println("Run "+n+" experiments for quality = "+quality+", average time: "+average);
        }

        logger.plot();
        logger.close();

        return 0;
    }

    private void printHelp() {
        HelpFormatter formatter = new HelpFormatter();
        formatter.printHelp( "ExperimentCreator", commandLineOptions);
    }

    private void parseCommandLine(CommandLine c) throws ParseException {
        start = Double.parseDouble(c.getOptionValue("start"));
        step = Double.parseDouble(c.getOptionValue("step", "1"));
        end = Double.parseDouble(c.getOptionValue("end"));

        log =  c.getOptionValue("log");

        n = Integer.parseInt(c.getOptionValue("n", "1"));

        verbose = c.hasOption("v");
//        verbose = true;
    }

    /**
     * Initialize command line options
     */
    private void initCommandLineOptions() {
        optionBuilder
                // Boolean options
                .add("h", "Show help", false, false)
                .add("v", "Show more verbose output", false, false)

                        // Options with value
                .add("log", "Save output to given file", true, false)
                .add("n", "[1] Run each experiment n-times and produce mean measurements", true, false)
                .add("start", "* Start value for the measured quality", true, true)
                .add("step", "[1] Step for increasing the measured quality", true, false)
                .add("end", "* End value for the measured quality", true, true)
                .add("type", "* Type of the experiment. Available: AgentScaling, SpeedScaling, GraphScaling", true, true);

        // Experiment-specific options
        initExperimentOptions();
    }




    /**
     * Initialize experiment-specific options
     */
    protected abstract void initExperimentOptions();

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
