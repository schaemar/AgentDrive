package cz.agents.agentdrive.simulator.lite.creator;

import cz.agents.agentdrive.highway.agent.Agent;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.LocalSimulatorHandler;
import cz.agents.agentdrive.highway.environment.SimulatorHandlers.ModuleSimulatorHandler;
import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.highway.storage.HighwayStorage;
import cz.agents.agentdrive.highway.storage.plan.PlansOut;
import cz.agents.agentdrive.highway.util.Utils;
import cz.agents.agentdrive.simulator.lite.environment.SimulatorEnvironment;
import cz.agents.agentdrive.simulator.lite.visualization.*;
import cz.agents.agentdrive.simulator.lite.visualization.SimulationControlLayer;
import cz.agents.alite.configreader.ConfigReader;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.creator.Creator;
import cz.agents.alite.creator.CreatorFactory;
import cz.agents.alite.simulation.Simulation;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.layer.common.*;

import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import javax.vecmath.Point2d;
import java.awt.*;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.*;

/**
 * Creates and initializes the simulation environment and the Alite event processor
 *
 * Created by wmatex on 3.7.14.
 */
public class SimulatorCreator implements Creator {
    private Simulation simulation;
    protected long timestep;
    private SimulatorEnvironment environment;
    private static String CONFIG_FILE = "settings/groovy/config.groovy";
    protected String DEFAULT_CONFIG_FILE = "settings/groovy/config.groovy";

    private static String HIGHWAY_CONFIG_FILE;
    private static final Logger logger = Logger.getLogger(SimulatorCreator.class);

    @Override
    public void init(String[] args) {
        if(args.length>1){
            CONFIG_FILE = args[1];
        }

        // Configuration loading using alite's Configurator and ConfigReader
        ConfigReader configReader = new ConfigReader();
        //configReader.loadAndMerge(DEFAULT_CONFIG_FILE);
        configReader.loadAndMerge(CONFIG_FILE);

        Configurator.init(configReader);


        String logfile = Configurator.getParamString("cz.highway.configurationFile", "settings/log4j/log4j.properties");
        PropertyConfigurator.configure(logfile);

        logger.setLevel(Level.INFO);
        logger.info("Configuration loaded from: " + CONFIG_FILE);
        if(logger.isDebugEnabled()){
            logger.debug("Printing complete configuration on the System.out >>");
            configReader.writeTo(new PrintWriter(System.out));
        }
        logger.info("log4j logger properties loaded from: " + logfile);
    }

    @Override
    public void create() {
        timestep = Configurator.getParamInt("simulator.lite.timestep", 100);
        simulation = new Simulation();
        double simulationSpeed = Configurator.getParamDouble("simulator.lite.simulationSpeed", 1.0);
        simulation.setSimulationSpeed(simulationSpeed);
        environment = new SimulatorEnvironment(simulation);
        environment.init();

        createVisualization();
        runSimulation();
    }

    private void createVisualization() {
        VisManager.setInitParam(Configurator.getParamString("simulator.lite.name","Simulator-Lite"), 1024, 768);
        VisManager.setSceneParam(new VisManager.SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(0, 0);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 3;
            }
        });

        VisManager.init();


        // Overlay
        //VisManager.registerLayer(ColorLayer.create(Color.LIGHT_GRAY));
        VisManager.registerLayer(ColorLayer.create(new Color(210, 255, 165)));
        VisManager.registerLayer(FpsLayer.create());
        try {
            VisManager.registerLayer(LogoLayer.create(Utils.getResourceUrl("img/atg_blue.png")));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        VisManager.registerLayer(VisInfoLayer.create());

        VisManager.registerLayer(HelpLayer.create());

        // Traffic visualization layer
        if(Configurator.getParamBool("simulator.lite.vis.SimulationControlLayer", false))
            VisManager.registerLayer(SimulationControlLayer.create(simulation, environment.getHighwayEnvironment()));
        if(Configurator.getParamBool("simulator.lite.vis.NetVisLayer", false))
            VisManager.registerLayer(NetVisLayer.create());
        if(Configurator.getParamBool("simulator.lite.vis.TrafficVisLayer", false))
            VisManager.registerLayer(TrafficVisLayer.create(environment.getStorage()));
        if(Configurator.getParamBool("simulator.lite.vis.ZoomVehicleLayer", false))
            VisManager.registerLayer(ZoomVehicleLayer.create(environment.getStorage()));
        if(Configurator.getParamBool("simulator.lite.vis.AgentDriveVisLayer", false))
            VisManager.registerLayer(AgentDriveVisLayer.create(environment.getHighwayEnvironment().getStorage()));
        if(Configurator.getParamBool("simulator.lite.vis.RoadObjectLayer", false))
            VisManager.registerLayer(RoadObjectLayer.create(environment.getHighwayEnvironment().getStorage().getPosCurr(),environment.getHighwayEnvironment().getStorage().getActions()));

    }

    public void runSimulation() {
        //System.out.println(simulation.getEventCount());

         initTraffic();
        logger.info("Simulation is ready to run.");
        simulation.run();

    }

    public static void main(String[] args) {
        SimulatorCreator creator = (SimulatorCreator) CreatorFactory.createCreator(args);
        creator.init(args);
        creator.create();
     //   creator.runSimulation();
    }


    private void initTraffic(){
        final XMLReader reader = new XMLReader(Configurator.getParamString("simulator.net.folder", "notDefined"));
        // All vehicle id's
        final Collection<Integer> vehicles = reader.getRoutes().keySet();
        final Map<Integer, Float> departures = reader.getDepartures();
        // final int size = vehicles.size();
        final int size;
        if (!Configurator.getParamBool("highway.dashboard.sumoSimulation", true)) {
            size = Configurator.getParamInt("highway.dashboard.numberOfCarsInSimulation", vehicles.size());
        } else {
            size = vehicles.size();
        }
        final int simulatorCount = Configurator.getParamList("highway.dashboard.simulatorsToRun", String.class).size();
        final HighwayStorage storage = environment.getHighwayEnvironment().getStorage();

        Iterator<Integer> vehicleIt = vehicles.iterator();
        PlansOut plans = new PlansOut();
        //   RadarData update = ;
        Map<Integer, Agent> agents = storage.getAgents();
        Set<Integer> plannedVehiclesLocal = new HashSet<Integer>();
        int sizeL = size;
        if (size > vehicles.size()) sizeL = vehicles.size();

        for (int i = 0; i < sizeL; i++) {
            int vehicleID = vehicleIt.next();
            if (Configurator.getParamBool("highway.dashboard.sumoSimulation", true)) {
                storage.addForInsert(vehicleID, departures.get(vehicleID));
            } else {
                storage.addForInsert(vehicleID);
            }
            plannedVehiclesLocal.add(vehicleID);
        }
        final Set<Integer> plannedVehicles = plannedVehiclesLocal;
      //  environment.getHighwayEnvironment().addSimulatorHandler(new LocalSimulatorHandler(environment.getHighwayEnvironment(), new HashSet<Integer>(plannedVehicles)));
        environment.getHighwayEnvironment().addSimulatorHandler(new ModuleSimulatorHandler(environment.getHighwayEnvironment(), new HashSet<Integer>(plannedVehicles), environment.getPlanCallback()));
    }


}
