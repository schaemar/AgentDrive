package cz.agents.highway.vis;

import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.element.*;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.GroupLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.terminal.LineLayer;
import cz.agents.alite.vis.layer.terminal.PointLayer;
import cz.agents.highway.environment.roadnet.Edge;
import cz.agents.highway.environment.roadnet.Junction;
import cz.agents.highway.environment.roadnet.Lane;
import cz.agents.highway.environment.roadnet.Network;
import java.awt.*;
import java.awt.Point;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.LinkedList;
import javax.vecmath.Point2f;
import javax.vecmath.Vector2f;



/**
 * Created by martin on 8.7.14.
 */
public class NetLayer extends GroupLayer implements VisLayer {
    
    final int LANE_WIGTH = 10;
    final int EDGE_WIGTH = 1;
    final float MAX_ANGLE = 0.1f;
    
    final Vector2f VECT_X_NORM = new Vector2f(1, 0);
    final Vector2f VECT_Y_NORM = new Vector2f(0, 1);
    
    

    private Network net;
    private Dimension dim = Vis.getDrawingDimension();
    private Rectangle2D drawingRectangle = new Rectangle(dim);

    public NetLayer(Network roadNetwork) {
        net = roadNetwork;
    }

    void paintLane(Graphics2D canvas, Point2f p1, Point2f p2){
        if(p1.x != p2.x && p1.y != p2.x){            
            canvas.setColor(Color.black);
            canvas.setStroke(new BasicStroke(LANE_WIGTH, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));            
            
            paintLine(canvas, p1, p2);
        }
    }
    
    void paintCircle(Graphics2D canvas, Point2f p, int size){
        AffineTransform t = canvas.getTransform();
        int x = Vis.transX(p.x);
        int y = Vis.transY(p.y);
        canvas.translate(x, y);
        canvas.scale(Vis.getZoomFactor(), Vis.getZoomFactor());
        
        float offset = size / 2 ;
        canvas.fillOval((int) (- offset),(int) (- offset), size, size);
        canvas.setTransform(t);
    }
    
    void paintLine(Graphics2D canvas, Point2f p1, Point2f p2){
        AffineTransform t = canvas.getTransform();
        int x = Vis.transX(p1.x);
        int y = Vis.transY(p1.y);
        float divX = p2.x - p1.x;
        float divY = p2.y - p1.y;

        canvas.translate(x, y);
        canvas.scale(Vis.getZoomFactor(), Vis.getZoomFactor());
        Line2D line2d = new Line2D.Double(0, 0, divX, divY);
        if (line2d.intersects(drawingRectangle) && !(divX == 0 && divY == 0)) {
            canvas.draw(line2d);
        }
        canvas.setTransform(t);
    }

    @Override
    public void paint(Graphics2D canvas) {
        super.paint(canvas);
        
        for (Lane lane : net.getLanes().values()) {
            Point2f prev = lane.getShape().get(0);
            for (Point2f point : lane.getShape()) {
                paintLane(canvas,point,prev);
                prev = point;
            }
            for (Lane outgoingLane : lane.getOutgoingLanes()) {
                Point2f point = outgoingLane.getShape().get(0);
                paintLane(canvas,prev,point);
            }
        }
    }
    
//    @Override
//    public void paint(Graphics2D canvas) {
//        super.paint(canvas);
//        int radius = 10;
//        
//        canvas.setColor(Color.black);
//        canvas.setStroke(new BasicStroke(LANE_WIGTH));
//        for (Lane lane : net.getLanes().values()) {
//            Point2f prev = lane.getShape().get(0);
//            for (Point2f point : lane.getShape()) {
//                paintLane(canvas,point,prev);
//                prev = point;
//            }
//            for (Lane outgoingLane : lane.getOutgoingLanes()) {
//                Point2f point = outgoingLane.getShape().get(0);
//                paintLane(canvas,prev,point);
//            }
//        }
//        
//        canvas.setColor(Color.red);
//        canvas.setStroke(new BasicStroke(EDGE_WIGTH));
//        for (Edge edge : net.getEdges().values()) {
//            if(edge.getShape().isEmpty())continue;
//            Point2f prev = edge.getShape().get(0);
//            for (Point2f point : edge.getShape()) {
//                paintLine(canvas,point,prev);
//                prev = point;
//            }
//        }
//
//        canvas.setColor(Color.green);
//        canvas.setStroke(new BasicStroke(2));
//
//        for (Junction junction : net.getJunctions().values()) {
//
//            if(junction.getShape().isEmpty())continue;
//            //
//            int size = junction.getShape().size();
//            Point2f prev = junction.getShape().get(size-1);
//            for (Point2f point : junction.getShape()) {
//                paintLine(canvas,point,prev);
//                prev = point;
//            }
//        }
//    }
}
