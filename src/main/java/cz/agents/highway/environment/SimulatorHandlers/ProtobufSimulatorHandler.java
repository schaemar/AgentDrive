package cz.agents.highway.environment.SimulatorHandlers;

import cz.agents.alite.protobuf.factory.ProtobufFactory;
import cz.agents.highway.environment.HighwayEnvironment;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.RoadObject;

import java.io.IOException;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * Created by david on 9/11/15.
 */
public class ProtobufSimulatorHandler extends SimulatorHandler {
    protected final ProtobufFactory factory;
    public ProtobufSimulatorHandler(HighwayEnvironment highwayEnvironment, Set<Integer> plannedVehicles, ProtobufFactory factory)
    {
        super(highwayEnvironment,plannedVehicles);
        this.factory = factory;
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
        try {
            factory.send(plans);
            factory.send(radarData);
            //  System.out.println("po zpracování " + System.currentTimeMillis());

        } catch (IOException e) {
            e.printStackTrace();
        }

        plans.clear();
    }

}
