package cz.agents.agentdrive.highway.environment.SimulatorHandlers;


import cz.agents.agentdrive.highway.storage.plan.PlansOut;

import java.util.HashSet;
import java.util.Observable;
import java.util.Set;

/**
 * Created by david on 7/13/16.
 */
public abstract class PlanCallback extends Observable {
    private Set<Integer> plannedVehiclesToRemove = new HashSet<>();
    public abstract void execute(PlansOut plans);
    public void addToPlannedVehiclesToRemove(Integer id){
        plannedVehiclesToRemove.add(id);
    }
    public void clearPlannedVehicles(){
        plannedVehiclesToRemove.clear();
    }
    public Set<Integer> getPlannedVehiclesToRemove(){
        return plannedVehiclesToRemove;
    }
}
