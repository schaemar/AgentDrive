package cz.agents.highway.protobuf.factory;

import java.util.Collection;

import com.google.protobuf.InvalidProtocolBufferException;

import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.highway.protobuf.generated.MessageContainer.Message;
import cz.agents.highway.protobuf.generated.PlanMessage.Action;
import cz.agents.highway.protobuf.generated.PlanMessage.Plan;
import cz.agents.highway.protobuf.generated.PlanMessage.Plans;
import cz.agents.highway.protobuf.generated.PlanMessage.Plans.Builder;
import cz.agents.highway.storage.plan.ComboAction;
import cz.agents.highway.storage.plan.PlansOut;


public class PlansFactory implements FactoryInterface<PlansOut,Message> {

	public Message encode(Object object) {
	    PlansOut plansOut = (PlansOut)object; 
	    
	   Builder plansBuilder = Plans.newBuilder();
	    
	    
	    for (int carId : plansOut.getCarIds()) {
            Collection<cz.agents.highway.storage.plan.Action> plan = plansOut.getPlan(carId);
           cz.agents.highway.protobuf.generated.PlanMessage.Plan.Builder planBuilder = Plan.newBuilder();
           planBuilder.setVehicleId(carId);
            for (cz.agents.highway.storage.plan.Action act : plan) {
                ComboAction action = (ComboAction)act;
                Action protoaction = Action.newBuilder().setStartTime(action.getTimeStamp()).setDuration((int)(action.getDuration()*1000))
                        .setLane(action.getLane()).setSpeed(action.getSpeed()).setWpx(action.getWpx()).setWpy(action.getWpy()).build();
                planBuilder.addActions(protoaction);
            }
            Plan protoplan = planBuilder.build();
            plansBuilder.addPlans(protoplan);
        }
	    
	   
	   
	    Plans plans = plansBuilder.build();
		return Message.newBuilder().setPlans(plans).build();
	}

	public PlansOut decode(Message pm)throws InvalidProtocolBufferException {
		if(!pm.hasPlans()){
			throw new InvalidProtocolBufferException("Trying to decode Plans with no Plans defined!");
		}
		PlansOut plans = new PlansOut();		
		
		Plans protoPlans = pm.getPlans();
		
		for (Plan plan : protoPlans.getPlansList()) {
            for (Action protoaction : plan.getActionsList()){
                cz.agents.highway.storage.plan.Action action = new cz.agents.highway.storage.plan.ComboAction(plan.getVehicleId(), protoaction.getSpeed(), protoaction.getLane(), protoaction.getDuration(), protoaction.getStartTime(),protoaction.getWpx(),protoaction.getWpy());


                plans.addAction(action);
            }
        }
		return plans;
	}

	public Class<PlansOut> getObjectClass() {
		return PlansOut.class;
	}

}
