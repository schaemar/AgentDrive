package cz.agents.agentdrive.roadnet;

import cz.agents.agentdrive.highway.environment.roadnet.Edge;
import cz.agents.agentdrive.highway.environment.roadnet.Lane;
import cz.agents.agentdrive.highway.environment.roadnet.LaneImpl;
import cz.agents.agentdrive.highway.environment.roadnet.network.NetworkLocation;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import org.junit.Before;
import org.junit.Test;
import tt.euclid2d.region.Rectangle;

import javax.vecmath.Point2f;
import javax.vecmath.Vector2f;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.HashMap;

import static org.junit.Assert.*;


public class RoadNetUsage {

    private final String testRoadNetPath = "roadnets/simple/";
    RoadNetwork roadNet;


    @Before
    public void loadNetwork() throws URISyntaxException {
        roadNet = XMLReaderTests.loadSimpleNetwork(testRoadNetPath);
    }

    @Test
    public void networkLocationAPI() throws Exception {
        NetworkLocation location = roadNet.getNetworkLocation();
        Point2f offset = location.getOffset();
        Rectangle convertedBoundary = location.getConvertedBoundary();
        Rectangle origBoundary = location.getOrigBoundary();
        String projection = location.getProjection();

        assertNotNull(offset);
        assertNotNull(convertedBoundary);
        assertNotNull(origBoundary);
        assertNotNull(projection);
    }

    @Test
    public void networkEdgesAPI() throws Exception {
        HashMap<String, Edge> edges = roadNet.getEdges();
        String id = edges.keySet().iterator().next();
        Edge edge = edges.get(id);
        String edgeID = edge.getId();
        assertEquals("ID of edge should equal to the key in the edges map", edgeID, id);
        assertIsEdgeId(edgeID);

        String fromNodeId = edge.getFrom(); //TODO String? - shouldn't we distinguish node, edge, lane ids
        assertIsNodeID(fromNodeId);
        String toNodeId = edge.getTo();
        assertIsNodeID(toNodeId);

        int priority = edge.getPriority(); //TODO check the range
        ArrayList<Point2f> shape = edge.getShape();
        assertNotNull(shape); //shape of edge often not specified

        String type = edge.getType();// TODO String?
        assertIsType(type);
    }


    @Test
    public void networkLaneAPI() throws Exception{
        //TODO validate also structure
        Edge edge = getAnEdge();
        HashMap<String, LaneImpl> lanes = edge.getLanes();
        int numOfLanes = lanes.size();
        for (int i = 0; i < numOfLanes ; i++) {
            LaneImpl lane = edge.getLaneByIndex(i);
            assertNotNull(lane);
            testLane(lane);
        }

    }

    //TODO test also other parts of RoadNetwork and structure

    private void testLane(Lane lane) {
        String laneId = lane.getLaneId();
        int index = lane.getIndex();
        Edge parentEdge = lane.getParentEdge();
        assertNotNull("parentEdge should not be null", parentEdge);

        ArrayList<Point2f> shape = lane.getShape();
        assertIsShape(shape);
        Vector2f center = lane.getCenter();
        assertNotNull(center);

        float length = lane.getLength();
        float speed = lane.getSpeed();

        ArrayList<Point2f> innerPoints = lane.getInnerPoints();
        assertNotNull(innerPoints);

        ArrayList<Lane> incomingLanes = lane.getIncomingLanes();
        ArrayList<Lane> outgoingLanes = lane.getOutgoingLanes();
        assertNotNull(incomingLanes);
        assertNotNull(outgoingLanes);

        Lane laneLeft = lane.getLaneLeft();
        Lane laneRight = lane.getLaneRight();

        Edge edge = null;
        Lane nextLane = lane.getNextLane(edge);
        Lane previousLane = lane.getPreviousLane(edge);
        assertNull(nextLane);
        assertNull(previousLane);


    }

    private Edge getAnEdge() {
        HashMap<String, Edge> edges = roadNet.getEdges();
        return edges.values().iterator().next();
    }


    private void assertIsType(String type) {
        //TODO impl
    }

    private void assertIsNodeID(String nodeID) {
        //TODO impl
    }

    private void assertIsEdgeId(String edgeID) {
        //TODO impl
    }

    private void assertIsShape(ArrayList<Point2f> shape) {
        assertNotNull(shape);
        assertTrue(shape.size() >= 2);
    }

}
