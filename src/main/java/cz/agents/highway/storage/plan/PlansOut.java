package cz.agents.highway.storage.plan;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.Map;


public class PlansOut {
	
	private final Map<Integer, Collection<Action>> plans; 
	
	public PlansOut(PlansOut oldPlans) {
	    this();
        this.plans.putAll(oldPlans.plans);
    }

    public PlansOut() {
        this.plans =  new LinkedHashMap<Integer, Collection<Action>>();
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

}
