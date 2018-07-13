package cz.agents.highway.creator;

import java.awt.Color;
import java.io.FileNotFoundException;
import java.io.PrintWriter;

import javax.vecmath.Point2d;

import cz.agents.highway.vis.*;
import org.apache.log4j.Level;
import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;

import cz.agents.alite.configreader.ConfigReader;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.creator.Creator;
import cz.agents.alite.simulation.Simulation;
import cz.agents.alite.vis.VisManager;
import cz.agents.alite.vis.VisManager.SceneParams;
import cz.agents.alite.vis.layer.common.ColorLayer;
import cz.agents.alite.vis.layer.common.FpsLayer;
import cz.agents.alite.vis.layer.common.HelpLayer;
import cz.agents.alite.vis.layer.common.LogoLayer;
import cz.agents.alite.vis.layer.common.VisInfoLayer;
import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.util.Utils;

public class DefaultCreator implements Creator {
    protected String DEFAULT_CONFIG_FILE = "settings/groovy/highway.groovy";
    protected String CONFIG_FILE = DEFAULT_CONFIG_FILE;

    protected Simulation simulation = null;
    protected HighwayEnvironment highwayEnvironment = null;

    private final Logger logger = Logger.getLogger(DefaultCreator.class);

    public void init(String[] args) {
        if(args.length>1){
            CONFIG_FILE = args[1];
        }        

        // Configuration loading using alite's Configurator and ConfigReader
        ConfigReader configReader = new ConfigReader();
        configReader.loadAndMerge(DEFAULT_CONFIG_FILE);
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

    public void create() {
        int seed = Configurator.getParamInt("highway.seed", 0);
        logger.info("Seed set to " + seed);
        double simulationSpeed = Configurator.getParamDouble("highway.simulationSpeed", 1.0);
        logger.info("Simulation speed: " + simulationSpeed);
        long simulationDuration = Configurator.getParamInt("highway.simulationDuration", -1);
        logger.info("Simulation duration: " + simulationDuration);

        logger.info(">>> SIMULATION CREATION");
        if(simulationDuration==-1){
            simulation = new Simulation();
        }else {
            simulation = new Simulation(simulationDuration);
        }
        logger.info(">>> ENVIRONMENT CREATION");
        highwayEnvironment = new HighwayEnvironment(simulation);

        if (Configurator.getParamBool("highway.vis.isOn", false)) {
            logger.info(">>> VISUALISATION CREATION");
            createVisualization();
            VisManager.registerLayer(new NetLayer(highwayEnvironment.getRoadNetwork()));
            VisManager.registerLayer(ProtobufVisLayer.create(highwayEnvironment.getStorage()));
            VisManager.registerLayer(RoadObjectLayer.create(highwayEnvironment.getStorage().getPosCurr(),highwayEnvironment.getStorage().getActions()));
            VisManager.registerLayer(SimulationControlLayer.create(simulation, highwayEnvironment));
           // VisManager.registerLayer(PlansLayer.create(highwayEnvironment.getStorage()));
        }
        simulation.setSimulationSpeed(simulationSpeed);
 
    }
    public void runSimulation(){
         simulation.run();
    }

    protected void createVisualization() {

        VisManager.setInitParam("Highway Protobuf Operator", 1024, 768);
        VisManager.setSceneParam(new VisManager.SceneParams() {

            @Override
            public Point2d getDefaultLookAt() {
                return new Point2d(0, 0);
            }

            @Override
            public double getDefaultZoomFactor() {
                return 0.75;
            }
        });

        VisManager.init();

        // Overlay

        VisManager.registerLayer(ColorLayer.create(Color.LIGHT_GRAY));
        VisManager.registerLayer(VisInfoLayer.create());
        VisManager.registerLayer(FpsLayer.create());
        try {
            VisManager.registerLayer(LogoLayer.create(Utils.getResourceUrl("img/atg_blue.png")));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        VisManager.registerLayer(HelpLayer.create());
        logger.info(">>> VISUALIZATION CREATED");

    }

    @Deprecated
    public static void main(String[] args) {
        System.out.print("RUNNING Highway DefaultCreator.java");
        for (int i = 0; i < args.length; i++) {
            System.out.print(" " + args[i]);
        }
        System.out.println(".");
        DefaultCreator creator = new DefaultCreator();
        creator.init(args);
        creator.create();
        creator.runSimulation();

    }

  

  

   

   
    
}
