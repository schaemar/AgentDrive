package cz.agents.highway.storage;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import cz.agents.alite.configurator.Configurator;
import cz.agents.highway.agent.*;
import cz.agents.highway.environment.HighwayEnvironment;
import org.apache.log4j.Logger;

import cz.agents.alite.common.event.Event;
import cz.agents.alite.environment.eventbased.EventBasedEnvironment;
import cz.agents.alite.environment.eventbased.EventBasedStorage;
import cz.agents.alite.simulation.SimulationEventType;
import cz.agents.highway.storage.plan.Action;

public class HighwayStorage extends EventBasedStorage {

    private final Logger logger = Logger.getLogger(HighwayStorage.class);

    private final RoadDescription roadDescription;
    private final Map<Integer, Agent> agents = new LinkedHashMap<Integer, Agent>();
    private final Map<Integer, RoadObject> posCurr = new LinkedHashMap<Integer, RoadObject>();
    private final Map<Integer, Action> actions = new LinkedHashMap<Integer, Action>();

    private Agent queen;

    public HighwayStorage(HighwayEnvironment environment) {
        super(environment);
        environment.getEventProcessor().addEventHandler(this);
        roadDescription = new RoadDescription(environment.getRoadNetwork());

    }

    @Override
    public void handleEvent(Event event) {

        if (event.isType(SimulationEventType.SIMULATION_STARTED)) {
            logger.debug("HighwayStorage: handled simulation START");
        }else if(event.isType(HighwayEventType.RADAR_DATA)){
            logger.debug("HighwayStorage: handled: RADAR_DATA");
            RadarData radar_data = (RadarData) event.getContent();
            updateCars(radar_data);
        }

    }

    public void updateCar(RoadObject carState) {
        int carId = carState.getId();

//        if (!agents.containsKey(carId)) {
//            createAgent(carId);
//        }
        posCurr.put(carId, carState);
    }

    public Agent createAgent(final int id) {
        String agentClassName = Configurator.getParamString("highway.agent", "RouteAgent");
        Agent agent = null;
        if (agentClassName.equals("RouteAgent")) {
            agent = new RouteAgent(id);
        } else if (agentClassName.equals("SDAgent")) {
            agent = new SDAgent(id);
        } else if (agentClassName.equals("ORCAAgent")) {
            agent = new ORCAAgent(id);
        }else if (agentClassName.equals("test")) {
            agent = new testAgent(id);
        }

        VehicleSensor sensor = new VehicleSensor(getEnvironment(), agent, this);
        VehicleActuator actuator = new VehicleActuator(getEnvironment(), agent, this);
        agent.addSensor(sensor);
        agent.addActuator(actuator);

        agents.put(id, agent);
        return agent;
    }

    public void act(int carId, Action action) {
        actions.put(carId, action);

    }public void act(int carId, List<Action> action) {
        actions.put(carId, action.get(0));

    }

    public RoadDescription getRoadDescription() {
        return roadDescription;
    }

    public Map<Integer, Agent> getAgents() {
        return agents;
    }

    public Map<Integer, RoadObject> getPosCurr() {
        return posCurr;
    }

    public Map<Integer, Action> getActions() {
        return actions;
    }

    public void updateCars(RadarData object) {
        if(!object.getCars().isEmpty()) {
            for (RoadObject car : object.getCars()) {
                updateCar(car);
            }
            logger.debug("HighwayStorage updated vehicles: received " + object);
            getEventProcessor().addEvent(HighwayEventType.UPDATED, null, null, null);
        }
    }

//    public void updateInit(InitIn init) {
//        getRoadDescription().addPoints(init.getPoints());
//
//    }
}
