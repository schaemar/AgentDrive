package cz.agents.highway.protobuf.factory.simplan;

import java.util.Collection;

import javax.vecmath.Point3f;

import com.google.protobuf.InvalidProtocolBufferException;

import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.highway.protobuf.generated.simplan.MessageContainer.Message;
import cz.agents.highway.protobuf.generated.simplan.PlanMessage.Action;
import cz.agents.highway.protobuf.generated.simplan.PlanMessage.Actuators;
import cz.agents.highway.protobuf.generated.simplan.PlanMessage.Maneuver;
import cz.agents.highway.protobuf.generated.simplan.PlanMessage.Plan;
import cz.agents.highway.protobuf.generated.simplan.PlanMessage.Plans;
import cz.agents.highway.protobuf.generated.simplan.PlanMessage.Plans.Builder;
import cz.agents.highway.protobuf.generated.simplan.PlanMessage.Waypoint;
import cz.agents.highway.protobuf.generated.simplan.VectorProto.Vector;
import cz.agents.highway.storage.plan.ActuatorsAction;
import cz.agents.highway.storage.plan.ManeuverAction;
import cz.agents.highway.storage.plan.PlansOut;
import cz.agents.highway.storage.plan.WPAction;


public class PlansFactory implements FactoryInterface<PlansOut,Message> {

	public Message encode(Object object) {
	    PlansOut plansOut = (PlansOut)object; 
	    
	   Builder plansBuilder = Plans.newBuilder();
	   
	   double timeStamp = 0;
	    
	    for (int carId : plansOut.getCarIds()) {
            Collection<cz.agents.highway.storage.plan.Action> plan = plansOut.getPlan(carId);
           cz.agents.highway.protobuf.generated.simplan.PlanMessage.Plan.Builder planBuilder = Plan.newBuilder();
           planBuilder.setVehicleId(carId);
           
            for (cz.agents.highway.storage.plan.Action action : plan) {
                cz.agents.highway.protobuf.generated.simplan.PlanMessage.Action.Builder actBuilder = Action.newBuilder();
                
                
                if(action.getClass().equals(ActuatorsAction.class)){
                    ActuatorsAction a = (ActuatorsAction)action;
                    Actuators actuator = Actuators.newBuilder().setSteer(a.getSteer()).setGas(a.getGas()).setBrake(a.getBrake()).setDuration(a.getDuration()).build();
                    actBuilder.setActuators(actuator);
                }
                else if(action.getClass().equals(WPAction.class)){
                    WPAction a = (WPAction)action;
                    Point3f p = a.getPosition();
                    Vector pos = Vector.newBuilder().setX(p.x).setY(p.y).setZ(p.z).build();
                    Waypoint wp = Waypoint.newBuilder().setPosition(pos).setSpeed(a.getSpeed()).build();
                    actBuilder.setWaypoint(wp);
                    
                }else if(action.getClass().equals(ManeuverAction.class)){
                    ManeuverAction m = (ManeuverAction)action;
                    Maneuver man = Maneuver.newBuilder().setSpeed(m.getSpeed()).setLane(m.getLane()).setDuration(m.getDuration()).build();
                    actBuilder.setManeuver(man);
                }
               timeStamp = action.getTimeStamp();
               
               planBuilder.addActions(actBuilder);
            }
            Plan protoplan = planBuilder.build();
            plansBuilder.addPlans(protoplan);
        }
	    	   
	   
        plansBuilder.setTimestamp(timeStamp);
	    Plans plans = plansBuilder.build();
		return Message.newBuilder().setPlans(plans).build();
	}

	public PlansOut decode(Message pm)throws InvalidProtocolBufferException {
		if(!pm.hasPlans()){
			throw new InvalidProtocolBufferException("Trying to decode Plans with no Plans defined!");
		}
		PlansOut plans = new PlansOut();		
		
		Plans protoPlans = pm.getPlans();
		
		 double timeStamp = protoPlans.getTimestamp();
		
		for (Plan plan : protoPlans.getPlansList()) {
		    int id = plan.getVehicleId();
		   
            for (Action protoaction : plan.getActionsList()){
                cz.agents.highway.storage.plan.Action action = null;
                if(protoaction.hasActuators()){
                   Actuators a =  protoaction.getActuators();
                    action = new ActuatorsAction(id, timeStamp, a.getSteer(), a.getGas(),a.getBrake(), a.getDuration());
                }else if(protoaction.hasWaypoint()){
                    Waypoint wp = protoaction.getWaypoint();
                    Point3f pos = new Point3f((float)wp.getPosition().getX(),(float)wp.getPosition().getY(),(float)wp.getPosition().getZ());
                    action = new WPAction(id, timeStamp, pos, wp.getSpeed());                    
                }else if(protoaction.hasManeuver()){
                    Maneuver man = protoaction.getManeuver();
                    action = new ManeuverAction(id, timeStamp, man.getSpeed(), man.getLane(), man.getDuration());
                }

                plans.addAction(action);
            }
        }
		return plans;
	}

	public Class<PlansOut> getObjectClass() {
		return PlansOut.class;
	}

}
