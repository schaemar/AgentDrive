package cz.agents.highway.environment.roadnet;


import cz.agents.alite.configurator.Configurator;
import cz.agents.highway.environment.roadnet.network.RoadNetwork;
import cz.agents.highway.util.Utils;
import org.apache.log4j.Logger;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.vecmath.Point2f;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.io.*;
import java.util.*;

/**
 * XMLReader reads a .net.xml, .rou.xml file and creates a street network.
 * Note: connections are not parsed
 */
public class XMLReader {
    private static XMLReader instance = null;
    private static HashMap<String, Edge> edgeMap = new HashMap<String, Edge>();
    private static HashMap<String, Junction> junctionMap = new HashMap<String, Junction>();
    private static HashMap<String, LaneImpl> laneMap = new HashMap<String, LaneImpl>();
    private File netfile;
    private ArrayList<Connection> connectionList = new ArrayList<Connection>();
    private ArrayList<String> tunnels = new ArrayList<String>();
    private ArrayList<String> bridges = new ArrayList<String>();

    private final static Logger logger = Logger.getLogger(XMLReader.class);
    private HashMap<Integer, List<String>> routes;
    private final Map<Integer, Point2f> initialPositions = new HashMap<Integer, Point2f>();
    private final Map<Integer, Float> departures = new HashMap<Integer, Float>();


    private XMLReader() {

    }

    public XMLReader(File netFile) {
        this.netfile = netFile;
    }

    public static synchronized XMLReader getInstance() {
        if (instance == null) {
            instance = new XMLReader();
        }
        return instance;
    }

    /**
     * Parse all the network ( Lanes, Edges, Junctions, Routes
     *
     * @param networkFolder where *.net.xml, *.rou.xml (optionally bridges and tunnels) files are stored
     */
    public void read(String networkFolder) {
        logger.info("PARSING NETWORK");
        try {
            File networkFile = Utils.getFileWithSuffix(networkFolder, ".net.xml");
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();

            Document doc = dBuilder.parse(networkFile);

            NodeList edgeNodeList = doc.getElementsByTagName("edge");

            for (int temp = 0; temp < edgeNodeList.getLength(); temp++) {

                Node nNode = edgeNodeList.item(temp);

                if (nNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element e = (Element) nNode;

                    String id = e.getAttribute("id");
                    String from = e.getAttribute("from");
                    String to = e.getAttribute("to");
                    String priority = e.getAttribute("priority");
                    String type = e.getAttribute("type");
                    String shapeStr1 = e.getAttribute("shape");

                    Edge edge = new Edge(id, from, to, priority, type, getShape(shapeStr1));
                    NodeList laneNodeList = e.getElementsByTagName("lane");
                    HashMap<String, LaneImpl> lanes = parseLanes(laneNodeList);
                    edge.putLanes(lanes);
                    laneMap.putAll(lanes);

                    edgeMap.put(edge.getId(), edge);
                }
            }

            NodeList junctionNodeList = doc.getElementsByTagName("junction");
            for (int temp = 0; temp < junctionNodeList.getLength(); temp++) {

                Node jNode = junctionNodeList.item(temp);

                if (jNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element j = (Element) jNode;

                    String id = j.getAttribute("id");
                    String type = j.getAttribute("type");
                    float x = Float.valueOf(j.getAttribute("x"));
                    float y = Float.valueOf(j.getAttribute("y"));
                    Point2f center = transSUMO2Alite(x, y);
                    String incLanesStr = j.getAttribute("incLanes");

                    String intLanesStr = j.getAttribute("incLanes");
                    String shapeStr = j.getAttribute("shape");

                    NodeList requestNodeList = j.getElementsByTagName("request");
                    ArrayList<Request> requestList = new ArrayList<Request>();
                    for (int requestIndex = 0; requestIndex < requestNodeList.getLength(); requestIndex++) {

                        String index = j.getAttribute("index");
                        String response = j.getAttribute("response");
                        String foes = j.getAttribute("foes");
                        Request request = new Request(index, response, foes);
                        requestList.add(request);
                    }

                    Junction junction = new Junction(id, type, center, separateStrings(incLanesStr), separateStrings(intLanesStr), getShape(shapeStr), requestList);
                    junctionMap.put(junction.getId(), junction);
                }
            }

            NodeList connectionNodeList = doc.getElementsByTagName("connection");
            for (int temp = 0; temp < connectionNodeList.getLength(); temp++) {

                Node cNode = connectionNodeList.item(temp);

                if (cNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element c = (Element) cNode;

                    String from = c.getAttribute("from");
                    String to = c.getAttribute("to");
                    String fromLane = c.getAttribute("fromLane");
                    String toLane = c.getAttribute("toLane");

                    Connection connection = new Connection(from, to, fromLane, toLane);
                    connectionList.add(connection);
                }
            }

            parseMultilevelJunctions();
            routes = parseRoutes(Utils.getFileWithSuffix(networkFolder, ".rou.xml"));

            Network.getInstance().init(edgeMap, junctionMap, laneMap, connectionList, tunnels, bridges);
            logger.info("NETWORK PARSED");

        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * @param lanesNodeList nodeList of lanes
     * @return map LaneID (String) -> Lane
     */
    private HashMap<String, LaneImpl> parseLanes(NodeList lanesNodeList) {
        HashMap<String, LaneImpl> ret = new HashMap<String, LaneImpl>();
        for (int temp = 0; temp < lanesNodeList.getLength(); temp++) {

            Node lNode = lanesNodeList.item(temp);

            if (lNode.getNodeType() == Node.ELEMENT_NODE) {

                Element l = (Element) lNode;
                String laneId = l.getAttribute("id");
                int index = Integer.parseInt(l.getAttribute("index"));
                float speed = Float.valueOf(l.getAttribute("speed"));
                float length = Float.valueOf(l.getAttribute("length"));
                String shapeStr = l.getAttribute("shape");


                LaneImpl lane = new LaneImpl(laneId, index, speed, length, getShape(shapeStr));
                ret.put(laneId, lane);
            }
        }

        return ret;
    }

    /**
     * Parses routes (vehicles and routes)
     *
     * @param routesFile .rou.xml file
     * @return map vehicleID -> it's route (list of edge IDs)
     */
    private HashMap<Integer, List<String>> parseRoutes(File routesFile) {
        HashMap<Integer, List<String>> plans = new HashMap<Integer, List<String>>();
        logger.info("PARSING ROUTES");
        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();

            Document doc = dBuilder.parse(routesFile);

            NodeList edgeNodeList = doc.getElementsByTagName("vehicle");
            for (int temp = 0; temp < edgeNodeList.getLength(); temp++) {

                Node lNode = edgeNodeList.item(temp);

                if (lNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element l = (Element) lNode;
                    int id = Integer.parseInt(l.getAttribute("id"));
                    float depart = Float.valueOf(l.getAttribute("depart"));
                    departures.put(id, depart);
                    String initPosition = l.getAttribute("initialPosition");
                    if (initPosition != null && !initPosition.isEmpty()) {
                        Point2f initialPosition = getShape(initPosition).get(0);
                        initialPositions.put(id, initialPosition);
                    }
                    Element route = (Element) l.getElementsByTagName("route").item(0);
                    ArrayList<String> plan = separateStrings(route.getAttribute("edges"));
                    plans.put(id, plan);
                }
            }
            return plans;

        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return null;
    }

    private ArrayList<String> separateStrings(String inputString) {
        StringTokenizer st = new StringTokenizer(inputString);
        ArrayList<String> ret = new ArrayList<String>();
        while (st.hasMoreTokens()) {
            ret.add(st.nextToken());
        }
        return ret;
    }

    /**
     * Parse shape String to a list of Points
     *
     * @param shapeString shape points separated by spaces
     * @return list of shape points
     */
    private ArrayList<Point2f> getShape(String shapeString) {
        StringTokenizer st = new StringTokenizer(shapeString);
        ArrayList<Point2f> shape = new ArrayList<Point2f>();
        while (st.hasMoreTokens()) {
            String p = st.nextToken();
            StringTokenizer st2 = new StringTokenizer(p, ",");
            float x = Float.valueOf(st2.nextToken());
            float y = Float.valueOf(st2.nextToken());

            Point2f point = transSUMO2Alite(x, y);
            shape.add(point);
        }
        return shape;
    }


    private void parseMultilevelJunctions() {
        String folderPath = Configurator.getParamString("highway.net.folder", "nets/junction-big");
        try {
            File tunnelsFile = Utils.getFileWithSuffix(folderPath, "." + MultilevelJunctionEdge.tunnels.toString());
            File bridgesFile = Utils.getFileWithSuffix(folderPath, "." + MultilevelJunctionEdge.bridges.toString());
            parseJunctionAndBridgesFiles(tunnelsFile, bridgesFile);
        } catch (FileNotFoundException e) {
            logger.warn(e.getMessage());
            logger.warn("tunnels/bridges file not found, parsing osm file");
            parseOSMForTunnelsAndBridges(folderPath);
        }
    }

    private void parseJunctionAndBridgesFiles(File tunnelsFile, File bridgesFile) {
        if (!parseFileToList(tunnelsFile, tunnels) || !parseFileToList(bridgesFile, bridges)) {
            logger.warn("Parsing tunnels,bridges unsuccessful, parsing osm file");
            parseOSMForTunnelsAndBridges(tunnelsFile.getParent());
        }
    }

    private boolean parseFileToList(File file, ArrayList list) {
        BufferedReader br;
        try {
            br = new BufferedReader(new FileReader(file));
            String line;
            while ((line = br.readLine()) != null) {
                list.add(line);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return false;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

    private void parseOSMForTunnelsAndBridges(String folderPath) {
        File osmFile;
        try {
            osmFile = Utils.getFileWithSuffix(folderPath, ".osm");

            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();

            Document doc = dBuilder.parse(osmFile);

            NodeList wayList = doc.getElementsByTagName("way");

            for (int temp = 0; temp < wayList.getLength(); temp++) {

                Node wNode = wayList.item(temp);

                if (wNode.getNodeType() == Node.ELEMENT_NODE) {

                    Element w = (Element) wNode;
                    String id = w.getAttribute("id");

                    NodeList tagNodeList = w.getElementsByTagName("tag");

                    for (int t = 0; t < tagNodeList.getLength(); t++) {

                        Node tNode = tagNodeList.item(t);

                        if (tNode.getNodeType() == Node.ELEMENT_NODE) {

                            Element tag = (Element) tNode;
                            String key = tag.getAttribute("k");
                            if (key.equals(MultilevelJunctionEdge.tunnels.toString())) {
                                tunnels.add(id);
                                break;
                            }
                            if (key.equals(MultilevelJunctionEdge.bridges.toString())) {
                                bridges.add(id);
                                break;
                            }
                        }
                    }
                }
            }

        } catch (FileNotFoundException e) {
            logger.warn("osm file not found, exception caught");
        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }


    /**
     * @return map from vehicleID to its route
     */
    public HashMap<Integer, List<String>> getRoutes() {
        return routes;
    }

    public Map<Integer, Point2f> getInitialPositions() {
        return initialPositions;
    }


    /**
     * Transformation of SUMO to ALite coordinates
     *
     * @param x x in SUMO coordinates
     * @param y y in SUMO coordinates
     * @return coordinates in Alite coordinates (x,-y)
     */
    private Point2f transSUMO2Alite(float x, float y) {
        return new Point2f(x, -y);
    }

    public RoadNetwork parseNetwork() {
        //TODO implement
        return null;
    }


    private enum MultilevelJunctionEdge {
        tunnels, bridges
    }

    public Map<Integer, Float> getDepartures() {
        return departures;
    }
}
