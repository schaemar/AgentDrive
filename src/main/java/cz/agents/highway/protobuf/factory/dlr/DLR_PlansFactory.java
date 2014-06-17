package cz.agents.highway.protobuf.factory.dlr;

import java.util.Collection;

import com.google.protobuf.InvalidProtocolBufferException;

import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.highway.protobuf.generated.dlr.DLR_MessageContainer.Message;
import cz.agents.highway.protobuf.generated.dlr.DLR_PlanMessage.Action;
import cz.agents.highway.protobuf.generated.dlr.DLR_PlanMessage.Plan;
import cz.agents.highway.protobuf.generated.dlr.DLR_PlanMessage.Plans;
import cz.agents.highway.protobuf.generated.dlr.DLR_PlanMessage.Plans.Builder;
import cz.agents.highway.storage.plan.ManeuverAction;
import cz.agents.highway.storage.plan.PlansOut;

public class DLR_PlansFactory implements FactoryInterface<PlansOut, Message> {

	public Message encode(Object object) {
		PlansOut plansOut = (PlansOut) object;

		Builder plansBuilder = Plans.newBuilder();

		double timeStamp = 0;

		for (int carId : plansOut.getCarIds()) {
			Collection<cz.agents.highway.storage.plan.Action> plan = plansOut.getPlan(carId);
			Plan.Builder planBuilder = Plan.newBuilder();
			planBuilder.setVehicleId(carId);

			for (cz.agents.highway.storage.plan.Action action : plan) {
				ManeuverAction m = (ManeuverAction) action;

				//TODO hacked lane = lane - 1 to meet dlr lane indexing - check DLR_UpdateFactory.java too
				Action man = Action.newBuilder().setSpeed(m.getSpeed()).setLane(m.getLane()-1).setDuration((int) (m.getDuration() * 1000)).build();
				timeStamp = action.getTimeStamp();
				planBuilder.addActions(man);
			}
			Plan protoplan = planBuilder.build();
			plansBuilder.addPlans(protoplan);
		}

		plansBuilder.setTimeStamp(timeStamp);
		Plans plans = plansBuilder.build();
		return Message.newBuilder().setPlans(plans).build();
	}

	public PlansOut decode(Message pm) throws InvalidProtocolBufferException {
		if (!pm.hasPlans()) {
			throw new InvalidProtocolBufferException("Trying to decode Plans with no Plans defined!");
		}
		PlansOut plans = new PlansOut();

		Plans protoPlans = pm.getPlans();

		double timeStamp = protoPlans.getTimeStamp();

		for (Plan plan : protoPlans.getPlansList()) {
			int id = plan.getVehicleId();

			for (Action protoaction : plan.getActionsList()) {
				cz.agents.highway.storage.plan.Action action = null;
				
				action = new ManeuverAction(id, timeStamp, protoaction.getSpeed(), protoaction.getLane(), protoaction.getDuration());

				plans.addAction(action);
			}
		}
		return plans;
	}

	public Class<PlansOut> getObjectClass() {
		return PlansOut.class;
	}

}
