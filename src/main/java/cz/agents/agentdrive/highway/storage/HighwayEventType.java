package cz.agents.agentdrive.highway.storage;

public enum HighwayEventType implements cz.agents.alite.common.event.EventType{
    TIMESTEP,
    VEHICLE_UPDATE,
    UPDATED,
    NEW_PLAN,
    ORCA_UPDATED,
    PREFERRED_VELOCITY_UPDATED,
    RADAR_DATA,
    RVO_SIMULATOR_UPDATED,
    TRAJECTORY_UPDATED,
    TRAJECTORY_CHANGED
}
