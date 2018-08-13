package cz.agents.agentdrive.simulator.lite.storage;

import cz.agents.agentdrive.highway.storage.RadarData;
import cz.agents.agentdrive.highway.storage.RoadObject;
import cz.agents.agentdrive.simulator.lite.controller.ControllerInterface;
import cz.agents.agentdrive.simulator.lite.controller.PedalController;
import cz.agents.agentdrive.simulator.lite.environment.SimulatorEnvironment;
import cz.agents.agentdrive.simulator.lite.storage.vehicle.Ghost;
import cz.agents.agentdrive.simulator.lite.storage.vehicle.Vehicle;
import cz.agents.alite.common.event.Event;
import cz.agents.alite.environment.eventbased.EventBasedEnvironment;
import cz.agents.alite.environment.eventbased.EventBasedStorage;

import javax.vecmath.Vector3f;
import java.util.*;

/**
 * Stores all simulated vehicles and handles their updates
 *
 * Created by wmatex on 3.7.14.
 */
public class VehicleStorage extends EventBasedStorage {
    private Map<Integer, Vehicle> vehicles = new HashMap<Integer, Vehicle>();
    private Map<Integer, Ghost>   ghosts = new HashMap<Integer, Ghost>();
    private ControllerInterface controller = new PedalController();

    /// Time of the last update
    long lastUpdate;

    public VehicleStorage(EventBasedEnvironment environment) {
        super(environment);
        lastUpdate = 0;
    }

    public void addVehicle(Vehicle vehicle) {
        synchronized (vehicles) {
            vehicles.put(vehicle.getId(), vehicle);
        }
    }
    public void removeVehicle(Integer id)
    {
        synchronized (vehicles) {
            vehicles.remove(id);
        }
    }
    public void addGhost(Ghost ghost) {
        ghosts.put(ghost.getId(), ghost);
    }
    public Vehicle getVehicle(Integer idVehicle){
        return vehicles.get(idVehicle);
    }
    public Ghost getGhost(int ghostID) {
        return ghosts.get(ghostID);
    }

    @Override
    public void handleEvent(Event event) {
        if (event.isType(SimulatorEvent.UPDATE)) {
            long t = getEventProcessor().getCurrentTime();

            if (lastUpdate != 0) {
                long deltaTime = t - lastUpdate;

                // Update all vehicles
                synchronized (vehicles) {
                    for (Vehicle v : vehicles.values()) {
                        //controller.updateVehicleVelocity(v, deltaTime, );

                        /// Update vehicle velocity based on it's acceleration
                        v.getVelocityController().updateVelocity(deltaTime);

                        /// Update vehicle position
                        v.update(deltaTime);
                    }
                }
            }
            lastUpdate = t;

            // Do another update after given time
            getEventProcessor().addEvent(SimulatorEvent.UPDATE, null, null, null, SimulatorEnvironment.UPDATE_STEP);
        }
    }

    public Collection<Vehicle> getVehicles() {
        return vehicles.values();
    }

    public Collection<Ghost> getGhosts() {
        return ghosts.values();
    }

    public LinkedList<Vehicle> getAllVehicles(){
        LinkedList<Vehicle> allVehicles = new LinkedList<Vehicle>();
        allVehicles.addAll(vehicles.values());
        allVehicles.addAll(ghosts.values());
        return allVehicles;
    }

    /**
     * Generate radar data to be sent to the highway server
     */
    public RadarData generateRadarData() {
        RadarData data = new RadarData();
        for (Vehicle vehicle: getVehicles()) {
            Vector3f velocity = new Vector3f(vehicle.getHeading());
            velocity.scale(vehicle.getVelocity());
            RoadObject object = new RoadObject(vehicle.getId(), getEventProcessor().getCurrentTime(), vehicle.getLane(),
                    vehicle.getPosition(), velocity);
            data.add(object);
        }

        return data;
    }
}
