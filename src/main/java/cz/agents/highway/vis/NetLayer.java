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

import javax.vecmath.Point2f;
import java.awt.*;
import java.awt.Point;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;

/**
 * Created by martin on 8.7.14.
 */
public class NetLayer extends GroupLayer implements VisLayer {

    private Network net;
    private Dimension dim = Vis.getDrawingDimension();
    private Rectangle2D drawingRectangle = new Rectangle(dim);

    public NetLayer(Network roadNetwork) {
        net = roadNetwork;
    }

    void paintLine(Graphics2D canvas, Point2f p1, Point2f p2){
        int x = Vis.transX(p1.x);
        int y = Vis.transY(p1.y);
        int xTo = Vis.transX(p2.x);
        int yTo = Vis.transY(p2.y);
        Line2D line2d = new Line2D.Double(x, y, xTo, yTo);
        if (line2d.intersects(drawingRectangle)) {
            canvas.draw(line2d);
        }
    }

    @Override
    public void paint(Graphics2D canvas) {
        super.paint(canvas);
        int radius = 10;
        canvas.setColor(Color.black);
        canvas.setStroke(new BasicStroke(10));


        for (Lane lane : net.getLanes().values()) {
            Point2f prev = lane.getShape().get(0);
            for (Point2f point : lane.getShape()) {
                paintLine(canvas,point,prev);
                prev = point;
            }
            for (Lane outgoingLane : lane.getOutgoingLanes()) {
                Point2f point = outgoingLane.getShape().get(0);
                paintLine(canvas,prev,point);
            }


        }
        canvas.setColor(Color.red);
        canvas.setStroke(new BasicStroke(5));
        for (Edge edge : net.getEdges().values()) {
            if(edge.getShape().isEmpty())continue;
            Point2f prev = edge.getShape().get(0);
            for (Point2f point : edge.getShape()) {
                paintLine(canvas,point,prev);
                prev = point;
            }


        }

        canvas.setColor(Color.green);
        canvas.setStroke(new BasicStroke(2));

        for (Junction junction : net.getJunctions().values()) {

            if(junction.getShape().isEmpty())continue;
            //
            int size = junction.getShape().size();
            Point2f prev = junction.getShape().get(size-1);
            for (Point2f point : junction.getShape()) {
                paintLine(canvas,point,prev);
                prev = point;
            }



        }

    }
}
