package cz.agents.highway.environment.roadnet;



import javax.vecmath.Point2f;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * Structure holding data about an edge loaded from sumo .net.xml file
 * Created by pavel on 19.6.14.
 */
public class Edge extends Sector {

    private final String from;
    private final String to;
    private final int priority;
    private final HashMap<String, Lane> lanes;


    public Edge(String id, String from, String to, String priority, String type, ArrayList<Point2f> shape) {
        super(id, type, shape);
        this.from = from;
        this.to = to;
        this.priority = Integer.parseInt(priority);
        this.lanes = new HashMap<String, Lane>();
    }

    public void putLanes(HashMap<String, Lane> laneMap) {
        for (Map.Entry<String, Lane> entry : laneMap.entrySet()) {
            entry.getValue().setEdge(this);
            lanes.put(entry.getKey(), entry.getValue());
        }
    }

    public String getFrom() {
        return from;
    }

    public String getTo() {
        return to;
    }

    public int getPriority() {
        return priority;
    }

    public HashMap<String, Lane> getLanes() {
        return lanes;
    }

    /**
     * Returns lane by given index
     * @return null if the index is invalid
     */
    public Lane getLaneByIndex(int laneIdx) {
        return getLanes().get(String.format("%s_%d", getId(), laneIdx));
    }


}
