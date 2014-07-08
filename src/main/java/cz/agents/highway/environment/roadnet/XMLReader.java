package cz.agents.highway.environment.roadnet;


import cz.agents.alite.configurator.Configurator;
import org.apache.log4j.Logger;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.vecmath.Point2f;
import javax.vecmath.Vector2f;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.StringTokenizer;

/**
 * XMLReader reads a .net.xml file and creates a street network.
 * Note: connections are not parsed
 * Created by pavel on 19.6.14.
 */
public class XMLReader {
    private static XMLReader instance = null;
    private static HashMap<String, Edge> edgeMap = new HashMap<String, Edge>();
    private static HashMap<String, Junction> junctionMap = new HashMap<String, Junction>();
    private static HashMap<String, Lane> laneMap = new HashMap<String, Lane>();
    private ArrayList<Connection> connectionList = new ArrayList<Connection>();
    private ArrayList<String> tunnels = new ArrayList<String>();
    private ArrayList<String> bridges = new ArrayList<String>();

    private final static Logger log = Logger.getLogger(XMLReader.class);



    private XMLReader() {

    }

    public static synchronized XMLReader getInstrance() {
        if (instance == null) {
            instance = new XMLReader();
        }
        return instance;
    }

    public void read(String networkFileName) {
        log.info("PARSING NETWORK");
        try {
            File fXmlFile = new File(networkFileName);
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder dBuilder = null;

            dBuilder = dbFactory.newDocumentBuilder();

            Document doc = dBuilder.parse(fXmlFile);

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

                    ArrayList<Lane> edgeLanes = new ArrayList<Lane>();
                    Edge edge = new Edge(id, from, to, priority, type, getShape(shapeStr1));
                    NodeList laneNodeList = e.getElementsByTagName("lane");
                    HashMap<String, Lane> lanes = parseLanes(laneNodeList);
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
                    Point2f center = new Point2f(x, y);
                    String incLanesStr = j.getAttribute("incLanes");

                    String intLanesStr = j.getAttribute("incLanes");
                    String shapeStr = j.getAttribute("shape");

                    NodeList requestNodeList = j.getElementsByTagName("request");
                    ArrayList<Request> requestList = new ArrayList<Request>();
                    for (int requestIndex = 0; requestIndex < requestNodeList.getLength(); requestIndex++) {

                        Node requestNode = requestNodeList.item(requestIndex);
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

            Network.getInstance().init(edgeMap, junctionMap, laneMap, connectionList, tunnels, bridges);
            log.info("NETWORK PARSED");

        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private HashMap<String, Lane> parseLanes(NodeList lanesNodeList) {
        HashMap<String, Lane> ret = new HashMap<String, Lane>();
        for (int temp = 0; temp < lanesNodeList.getLength(); temp++) {

            Node lNode = lanesNodeList.item(temp);

            if (lNode.getNodeType() == Node.ELEMENT_NODE) {

                Element l = (Element) lNode;
                String laneId = l.getAttribute("id");
                String index = l.getAttribute("index");
                float speed = Float.valueOf(l.getAttribute("speed"));
                float length = Float.valueOf(l.getAttribute("length"));
                String shapeStr = l.getAttribute("shape");


                Lane lane = new Lane(laneId, index, speed, length, getShape(shapeStr));
                ret.put(laneId, lane);
            }
        }

        return ret;
    }

    private ArrayList<String> separateStrings(String inputString) {
        StringTokenizer st = new StringTokenizer(inputString);
        ArrayList<String> ret = new ArrayList<String>();
        while (st.hasMoreTokens()) {
            ret.add(st.nextToken());
        }
        return ret;
    }

    private ArrayList<Point2f> getShape(String shapeString) {
        StringTokenizer st = new StringTokenizer(shapeString);
        ArrayList<Point2f> shape = new ArrayList<Point2f>();
        while (st.hasMoreTokens()) {
            String p = st.nextToken();
            StringTokenizer st2 = new StringTokenizer(p, ",");
            float x = Float.valueOf(st2.nextToken());
            float y = Float.valueOf(st2.nextToken());

            Point2f point = new Point2f(x, y);
            shape.add(point);
        }
        return shape;
    }

    private void parseMultilevelJunctions() {
        try {
            String folderPath = Configurator.getParamString("net.folder", "src/main/resources/nets/junction-big");
            String tunnelsFilePath = getFile(folderPath, "." + MultilevelJunctionEdge.tunnels.toString());
            String bridgesFilePath = getFile(folderPath, "." + MultilevelJunctionEdge.bridges.toString());
            if(tunnelsFilePath != null && bridgesFilePath != null){
                File tunnelsFile = new File(tunnelsFilePath);
                BufferedReader br = new BufferedReader(new FileReader(tunnelsFile));
                String line;
                while((line = br.readLine()) != null){
                    tunnels.add(line);
                }

                File bridgesFile = new File(bridgesFilePath);
                BufferedReader br2 = new BufferedReader(new FileReader(bridgesFile));
                String line2;
                while((line2 = br2.readLine()) != null){
                    bridges.add(line2);
                }
            }
            else {
                if(tunnelsFilePath == null){
                    log.error("tunnels file not found, parsing osm file");
                }
                if(bridgesFilePath == null){
                    log.error("bridges file not found, parsing osm file");
                }
                String osmFilePath = getFile(folderPath, ".osm");
                if (osmFilePath != null) {
                    File fXmlFile = new File(osmFilePath);
                    DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
                    DocumentBuilder dBuilder = null;


                    dBuilder = dbFactory.newDocumentBuilder();


                    Document doc = dBuilder.parse(fXmlFile);

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
                } else {
                    log.error("osm file not found, tunnels and bridges detection failed");
                }
            }
        } catch (ParserConfigurationException e) {
            e.printStackTrace();
        } catch (SAXException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private String getFile(String folderPath, String suffix) {
        File folder = new File(folderPath);
        if(folder.isDirectory()) {
            File[] files = folder.listFiles();
            for (File f : files) {
                if (f.getName().endsWith(suffix)) {
                    return f.getAbsolutePath();
                }
            }
        }
        return null;
    }

    private enum MultilevelJunctionEdge{
        tunnels, bridges
    }
}
