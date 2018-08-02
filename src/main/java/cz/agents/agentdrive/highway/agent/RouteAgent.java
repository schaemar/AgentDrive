package cz.agents.agentdrive.highway.agent;

import cz.agents.agentdrive.highway.maneuver.CarManeuver;
import cz.agents.agentdrive.highway.storage.HighwayEventType;
import cz.agents.agentdrive.highway.storage.VehicleSensor;
import cz.agents.agentdrive.highway.storage.plan.Action;
import cz.agents.alite.common.event.Event;
import cz.agents.alite.configurator.Configurator;
import org.apache.log4j.Logger;

import javax.vecmath.Point2f;
import javax.vecmath.Point3f;
import java.util.List;

/**
 * Created by martin on 9.7.14.
 */
public class RouteAgent extends Agent {

    private final static Logger logger = Logger.getLogger(RouteAgent.class);
    protected ManeuverTranslator maneuverTranslator;

    @Override
    public Point3f getInitialPosition() {

        //TODO  initial positioning with proper rotation
        Point2f p = navigator.getInitialPosition();
        return new Point3f(p.x, p.y, 0);
    }

    public RouteAgent(int id) {
        super(id);
        maneuverTranslator = new ManeuverTranslator(id, navigator);
    }

    public void addSensor(final VehicleSensor sensor) {
        this.sensor = sensor;
        maneuverTranslator.setSensor(this.sensor);
        this.sensor.registerReaction(new Reaction() {
            public void react(Event event) {
                if (event.getType().equals(HighwayEventType.UPDATED)) {
                    actuator.act(agentReact());
                }
            }
        });
    }

    /**
     * Generate an action as a reaction
     * Rout agent is adjusting speed according to the degrees of the curves and how many waypoints before himself will calculate.
     *
     * @return
     */
    protected List<Action> agentReact() {
        maneuverTranslator.setSensor(sensor);
        maneuverTranslator.setNavigator(navigator);
        return maneuverTranslator.prepare();
    }

    protected List<Action> agentReact(CarManeuver maneuver) {
        maneuverTranslator.setSensor(sensor);
        maneuverTranslator.setNavigator(navigator);
        return maneuverTranslator.translate(maneuver);
    }
}
