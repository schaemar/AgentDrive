package cz.agents.highway.storage.plan;

import java.util.*;


public class PlansOut {
	
	private final Map<Integer, Collection<Action>> plans; 
	
	public PlansOut(PlansOut oldPlans) {
	    this();
        this.plans.putAll(oldPlans.plans);
    }

    public PlansOut() {
        this.plans =  new LinkedHashMap<Integer, Collection<Action>>();
    }

    public void addActions(int id, List<Action> actions){
        Collection<Action> plan = plans.get(id);
        if(plan == null){
            plan = new LinkedList<Action>();
            plans.put(id, plan);
        }
        plan.addAll(actions);
    }

    public void addAction(Action action){
		Collection<Action> plan = plans.get(action.getCarId());
		if(plan == null){
			plan = new LinkedList<Action>();
			plans.put(action.getCarId(), plan);
		}
		plan.add(action);
	}
	
	public Collection<Action> getPlan(int carId) {
		return plans.get(carId);
	}
	
	public Collection<Integer> getCarIds() {
		return plans.keySet();
	}

    public void clear() {
       plans.clear();
        
    }

    @Override
    public String toString() {
        return plans.toString();
    }


}
