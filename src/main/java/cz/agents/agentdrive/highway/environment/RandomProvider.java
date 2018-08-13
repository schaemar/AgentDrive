package cz.agents.agentdrive.highway.environment;

import java.util.Random;

import org.apache.log4j.Logger;

import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.environment.Environment;

/* RandomProvider should be used for all random generating in the simulation
 *
 */
public final class RandomProvider {

    private final static Logger logger = Logger.getLogger(RandomProvider.class);

    //TODO better random generator could be used
    private static Random random;

    public static void init() {
        // Setting seed of random generator
        long seed = Configurator.getParamInt("simulator.lite.seed", -1);
        random = new Random(seed);
    }

    public static Random getRandom() {
        if (random == null) init();
        return random;
    }
}
