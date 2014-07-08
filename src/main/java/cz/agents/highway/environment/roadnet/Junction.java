package cz.agents.highway.environment.roadnet;



import javax.vecmath.Point2f;
import java.util.ArrayList;

/**
 * Structure holding data about a junction loaded from sumo .net.xml file
 * Created by pavel on 19.6.14.
 */
public class Junction extends Sector {
    private final ArrayList<Request> requests;
    private Point2f center;
    private ArrayList<String> incLanes;
    private ArrayList<String> intLanes;

    public Junction(String id, String type, Point2f center, ArrayList<String> incLanes,
                    ArrayList<String> intLanes, ArrayList<Point2f> shape, ArrayList<Request> requests) {
        super(id, type, shape);
        this.center = center;
        this.incLanes = incLanes;
        this.intLanes = intLanes;
        this.requests = requests;
    }


}
