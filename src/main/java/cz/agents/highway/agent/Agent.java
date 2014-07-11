package cz.agents.highway.agent;

import org.apache.log4j.Logger;

import cz.agents.alite.common.entity.Entity;
import cz.agents.highway.storage.VehicleActuator;
import cz.agents.highway.storage.VehicleSensor;

import javax.vecmath.Point3f;

public class Agent extends Entity {

    int id;
    private static final Logger logger = Logger.getLogger(Agent.class);

    protected VehicleSensor sensor;
    protected VehicleActuator actuator;
    

    public Agent(int id) {
        super("" + id);
        this.id = id;
        logger.info("Agent " + id + " created");
    }

    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        logger.info("Sensor added: " + sensor);
    }

    public void addActuator(VehicleActuator actuator) {
        this.actuator = actuator;
        logger.info("Actuator added: " + actuator);

    }

    public Point3f getInitialPosition() {
        return new Point3f(0,0,0);
    }

}
