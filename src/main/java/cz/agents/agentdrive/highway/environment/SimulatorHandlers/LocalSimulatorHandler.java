package cz.agents.agentdrive.highway.environment.SimulatorHandlers;

import cz.agents.alite.configurator.Configurator;
import cz.agents.agentdrive.highway.environment.HighwayEnvironment;
import cz.agents.agentdrive.highway.storage.HighwayEventType;
import cz.agents.agentdrive.highway.storage.RadarData;
import cz.agents.agentdrive.highway.storage.RoadObject;
import cz.agents.agentdrive.highway.storage.plan.Action;
import cz.agents.agentdrive.highway.storage.plan.PlansOut;
import cz.agents.agentdrive.highway.storage.plan.WPAction;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;
import java.util.Collection;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Created by david on 9/11/15.
 */
public class LocalSimulatorHandler extends SimulatorHandler {

    public LocalSimulatorHandler(HighwayEnvironment highwayEnvironment,Set<Integer> plannedVehicles) {
        super(highwayEnvironment,plannedVehicles);
    }

    @Override
    public void sendPlans(Map<Integer, RoadObject> vehicleStates) {
        RadarData radarData = new RadarData();

        Set<Integer> notPlanned = new HashSet<Integer>(vehicleStates.keySet());
        notPlanned.removeAll(plannedVehicles);

        for (int id : notPlanned) {
            radarData.add(vehicleStates.get(id));
        }

        // Finally send plans and updates
        executePlans(plans);
        plans.clear();
    }
    private void executePlans(PlansOut plans) {
        Map<Integer, RoadObject> currStates = highwayEnvironment.getStorage().getPosCurr();
        RadarData radarData = new RadarData();
        float duration = 0;
        float lastDuration = 0;
        double timest = Configurator.getParamDouble("highway.SimulatorLocal.timestep", 1.0);
        float timestep = (float)timest;

        boolean removeCar = false;
        for (Integer carID : plans.getCarIds()) {
            Collection<Action> plan = plans.getPlan(carID);
            RoadObject state = currStates.get(carID);
            Point3f lastPosition = state.getPosition();
            Point3f myPosition = state.getPosition();
            for (Action action : plan) {
                if (action.getClass().equals(WPAction.class)) {
                    WPAction wpAction = (WPAction) action;
                    if(wpAction.getSpeed() == -1)
                    {
                        myPosition = wpAction.getPosition();
                        removeCar = true;
                    }
                    if (wpAction.getSpeed() < 0.001) {
                        duration += 0.1f;
                    } else {
                        myPosition = wpAction.getPosition();
                        lastDuration = (float) (wpAction.getPosition().distance(lastPosition) / (wpAction.getSpeed()));
                        duration += wpAction.getPosition().distance(lastPosition) / (wpAction.getSpeed());
                    }
                    // creating point between the waipoints if my duration is greater than the defined timestep
                    if (duration >= timestep) {

                        float remainingDuration = timestep - (duration - lastDuration);
                        float ration = remainingDuration / lastDuration;
                        float x = myPosition.x - lastPosition.x;
                        float y = myPosition.y - lastPosition.y;
                        float z = myPosition.z - lastPosition.z;
                        Vector3f vec = new Vector3f(x, y, z);
                        vec.scale(ration);

                        myPosition = new Point3f(vec.x + lastPosition.x, vec.y + lastPosition.y, vec.z + lastPosition.z);
                        break;
                    }
                    lastPosition = wpAction.getPosition();
                }
            }
            if(removeCar)
            {
                if(Configurator.getParamBool("highway.dashboard.sumoSimulation",true))
                    plannedVehicles.remove(carID);
                removeCar = false;
            }
            else
            {
                Vector3f vel = new Vector3f(state.getPosition());
                vel.negate();
                vel.add(myPosition);
                if (vel.length() < 0.0001) {
                    vel = state.getVelocity();
                    vel.normalize();
                    vel.scale(0.001f);
                }
                int lane = highwayEnvironment.getRoadNetwork().getClosestLane(myPosition).getIndex();
                state = new RoadObject(carID, highwayEnvironment.getEventProcessor().getCurrentTime(), lane, myPosition, vel);
                radarData.add(state);
                duration = 0;
            }
        }
        //send radar-data to storage with duration delay
        System.out.println(highwayEnvironment.getEventProcessor().getCurrentTime());
        highwayEnvironment.getEventProcessor().addEvent(HighwayEventType.RADAR_DATA, highwayEnvironment.getStorage(), null, radarData, Math.max(1, (long) (timestep * 1000)));
    }
}
