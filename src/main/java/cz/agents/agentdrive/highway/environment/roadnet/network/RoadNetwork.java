package cz.agents.agentdrive.highway.environment.roadnet.network;

import cz.agents.agentdrive.highway.environment.roadnet.*;

import javax.vecmath.Point3f;
import java.util.ArrayList;
import java.util.HashMap;

public interface RoadNetwork {

    NetworkLocation getNetworkLocation();

    HashMap<String, Edge> getEdges();

    HashMap<String, Junction> getJunctions();

    ArrayList<String> getBridges();

    HashMap<String, LaneImpl> getLanes();

    ArrayList<String> getTunnels();

    Lane getClosestLane(Point3f position);

    ActualLanePosition getActualPosition(Point3f position);
}





