package cz.agents.agentdrive.highway.environment.SimulatorHandlers;

import cz.agents.agentdrive.highway.environment.HighwayEnvironment;
import cz.agents.agentdrive.highway.storage.RoadObject;
import cz.agents.agentdrive.highway.storage.plan.Action;
import cz.agents.agentdrive.highway.storage.plan.PlansOut;

import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Created by david on 9/11/15.
 */
public abstract class SimulatorHandler {
    protected final Set<Integer> plannedVehicles;
    protected final HighwayEnvironment highwayEnvironment;
    protected PlansOut plans = new PlansOut();

    protected SimulatorHandler(HighwayEnvironment highwayEnvironment,Set<Integer> plannedVehicles) {
        this.plannedVehicles = plannedVehicles;
        this.highwayEnvironment = highwayEnvironment;
    }

    public boolean hasVehicle(int vehicleID) {
        return plannedVehicles.contains(vehicleID);
    }

    @Deprecated
    public void addAction(Action action) {
        plans.addAction(action);
    }

    public void addActions(int id, List<Action> actions) {
        plans.addActions(id, actions);
    }

    public boolean isReady() {
        return plans.getCarIds().size() >= highwayEnvironment.getStorage().getPosCurr().size();
    }

    public int numberOfVehicles() {
        return plannedVehicles.size();
    }

    public abstract void sendPlans(Map<Integer, RoadObject> vehicleStates);

}
