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

    public NetLayer(Network roadNetwork) {
        net = roadNetwork;
    }

    @Override
    public void paint(Graphics2D canvas) {
        super.paint(canvas);
        int radius = 10;
        canvas.setColor(Color.black);
        canvas.setStroke(new BasicStroke(10));
        Dimension dim = Vis.getDrawingDimension();
        Rectangle2D drawingRectangle = new Rectangle(dim);

        for (Lane lane : net.getLanes().values()) {
            Point2f prev = lane.getShape().get(0);
            for (Point2f point : lane.getShape()) {

                int x = Vis.transX(prev.x);
                int y = Vis.transY(prev.y);
                int xTo = Vis.transX(point.x);
                int yTo = Vis.transY(point.y);
                Line2D line2d = new Line2D.Double(x, y, xTo, yTo);
                if (line2d.intersects(drawingRectangle)) {
                    canvas.draw(line2d);
                }
                prev = point;
            }
            lane.


        }
        canvas.setColor(Color.red);
        canvas.setStroke(new BasicStroke(5));
        for (Edge edge : net.getEdges().values()) {
            if(edge.getShape().isEmpty())continue;
            Point2f prev = edge.getShape().get(0);
            for (Point2f point : edge.getShape()) {

                int x = Vis.transX(prev.x);
                int y = Vis.transY(prev.y);
                int xTo = Vis.transX(point.x);
                int yTo = Vis.transY(point.y);
                Line2D line2d = new Line2D.Double(x, y, xTo, yTo);
                if (line2d.intersects(drawingRectangle)) {
                    canvas.draw(line2d);
                }
                prev = point;
            }


        }

        canvas.setColor(Color.green);
        canvas.setStroke(new BasicStroke(2));

        for (Junction junction : net.getJunctions().values()) {

            if(junction.getShape().isEmpty())continue;
            Point2f prev = junction.getShape().get(0);
            for (Point2f point : junction.getShape()) {

                int x = Vis.transX(prev.x);
                int y = Vis.transY(prev.y);
                int xTo = Vis.transX(point.x);
                int yTo = Vis.transY(point.y);
                Line2D line2d = new Line2D.Double(x, y, xTo, yTo);
                if (line2d.intersects(drawingRectangle)) {
                    canvas.draw(line2d);
                }
                prev = point;
            }


        }

    }
}
