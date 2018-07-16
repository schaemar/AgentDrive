package cz.agents.agentdrive.roadnet;

import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.agentdrive.highway.util.Utils;
import org.junit.Test;

import java.io.FileNotFoundException;
import java.net.URISyntaxException;
import java.util.HashMap;
import java.util.List;

import static junit.framework.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

public class XMLReaderTests {
    private static final String testRoadNetFolder = "roadnets/simple/";
    private static final String testNetFilePath = "roadnets/simple/simple.net.xml";
    private final String testRoutesPath = "roadnets/simple/simple.rou.xml";

    @Test
    public void loadSimpleNetwork() throws URISyntaxException {
        RoadNetwork net = loadSimpleNetwork(testRoadNetFolder);
        assertNotNull("loaded network should not be null", net);
    }

    public static RoadNetwork loadSimpleNetwork(String roadNetPath) {
        XMLReader parser = new XMLReader();
        RoadNetwork net = parser.parseNetwork(roadNetPath );
        return net;
    }

    @Test
    public void loadSimpleRoutes() throws URISyntaxException, FileNotFoundException {
        XMLReader parser = new XMLReader();
        HashMap<Integer, List<String>> routes = parser.parseRoutes(Utils.getResourceFile(testRoutesPath));
        assertNotNull("loaded routes should not be null", routes);
    }

    @Test
    public void notLoadedXMLReader(){
        XMLReader reader = new XMLReader();
        assertNull(reader.getNetwork());
        assertNull(reader.getDepartures());
        assertNull(reader.getInitialPositions());
        assertNull(reader.getRoutes());
        assertFalse(reader.isNetworkLoaded());
    }
}
