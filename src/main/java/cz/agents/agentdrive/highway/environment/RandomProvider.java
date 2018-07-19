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

	private static Environment environment;

	public static void init(Environment environmentToUse) {
		environment = environmentToUse;
		// Setting seed of random generator
		long seed = Configurator.getParamInt("simulator.lite.seed", -1);
		if (seed != -1) {
			environment.getRandom().setSeed(seed);
		}
		logger.info(" RandomProvider initialized with the seed = " + seed);

	}

	public static Random getRandom() {
		if (environment == null) {
			throw new RuntimeException("Random provider not initialized, requires the environment to provide Random");
		}
		return environment.getRandom();

	}

}
