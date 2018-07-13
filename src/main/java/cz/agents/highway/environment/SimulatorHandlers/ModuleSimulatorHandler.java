package cz.agents.highway.environment.SimulatorHandlers;

import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.RoadObject;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Created by david on 7/13/16.
 */
public class ModuleSimulatorHandler extends SimulatorHandler {
    protected PlanCallback planCallback;
    public ModuleSimulatorHandler(HighwayEnvironment highwayEnvironment, Set<Integer> plannedVehicles,PlanCallback planCallback) {
        super(highwayEnvironment,plannedVehicles);
        this.planCallback = planCallback;

    }
    @Override
    public void sendPlans(Map<Integer, RoadObject> vehicleStates) {
        RadarData radarData = new RadarData();

        Set<Integer> notPlanned = new HashSet<Integer>(vehicleStates.keySet());
        plannedVehicles.removeAll(planCallback.getPlannedVehiclesToRemove());
        notPlanned.removeAll(plannedVehicles);
       // planCallback.clearPlannedVehicles();
        for (int id : notPlanned) {
            radarData.add(vehicleStates.get(id));
        }
        plans.setCurrStates(vehicleStates);
        // Finally send plans and updates
        planCallback.execute(plans);
        plans.clear();

    }
    public boolean hasVehicle(int id)
    {
        return true;
    }

}
