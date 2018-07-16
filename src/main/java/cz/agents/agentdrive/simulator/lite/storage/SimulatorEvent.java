package cz.agents.agentdrive.simulator.lite.storage;

import cz.agents.alite.common.event.EventType;

/**
 * All various simulation events
 *
 * Created by wmatex on 3.7.14.
 */
public enum SimulatorEvent implements EventType{
    UPDATE,               /// Update vehicle positions
    COMMUNICATION_UPDATE  /// Send and receive new data from coordination module
}
