package cz.agents.highway.vis;

import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.highway.agent.Agent;
import cz.agents.highway.agent.GSDAgent;
import cz.agents.highway.storage.HighwayStorage;
import cz.agents.highway.storage.RoadObject;
import cz.agents.highway.storage.plan.WPAction;
import cz.agents.highway.storage.plan.Action;


import java.awt.*;
import java.awt.geom.Line2D;
import java.util.*;

import static java.awt.geom.AffineTransform.*;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.util.List;
import javax.swing.*;
import javax.vecmath.Point3f;

/**
 * Created by david on 9/3/15.
 */
public class PlansLayer extends AbstractLayer {
    private final int ARR_SIZE = 1;
    private Map<Integer,Agent> agents;
    private Map<Integer,RoadObject> objects;
    private Map<Integer,List<Action>> actions;



    public PlansLayer(Map<Integer,Agent> agents,Map<Integer,RoadObject> objects,Map<Integer, List<Action>> actions) {
        this.agents = agents;
        this.objects = objects;
        this.actions = actions;
    }

    @Override
    public void paint(Graphics2D canvas)
    {
        paintPlans(canvas);
        drawArrow(canvas, 30, 300, 300, 190);
        paintWaipoints(canvas);
    }

    public void paintPlans(Graphics2D canvas) {
        for (Map.Entry<Integer, RoadObject> entry : objects.entrySet()) {
            int id = entry.getKey();

            Agent test = agents.get(id);
            if (test instanceof GSDAgent) {
                canvas.setColor(Color.RED);
                RoadObject roadObject = entry.getValue();
                AffineTransform t = canvas.getTransform();
                Point3f pos = roadObject.getPosition();
                canvas.translate(Vis.transX(pos.getX()), Vis.transY(pos.getY()));
                double vehicleRotation = Math.atan2(roadObject.getVelocity().getY(), roadObject.getVelocity().getX());
                canvas.scale(Vis.getZoomFactor(), Vis.getZoomFactor());
                canvas.setStroke(new BasicStroke(0.1f));
                //canvas.drawLine(0, 0, (int) roadObject.getVelocity().x, (int) roadObject.getVelocity().y);
                drawArrow(canvas,0,0,(int) roadObject.getVelocity().x, (int) roadObject.getVelocity().y);
                //canvas.setColor(Color.BLUE);
                //canvas.rotate(vehicleRotation);
                canvas.setTransform(t);

            }
        }
    }
    public void paintWaipoints(Graphics2D canvas)
    {
        for (Map.Entry<Integer, List<Action>> entry : actions.entrySet()) {
            if(entry.getValue().get(0) instanceof WPAction)
            {
                for(int i =0;i<entry.getValue().size();i++)
                {
                    WPAction temp =(WPAction)entry.getValue().get(i);
                    paintCircle(canvas,temp.getPosition(), 1);
                }
            }
        }
    }
    void paintCircle(Graphics2D canvas, Point3f p, int size){
        AffineTransform t = canvas.getTransform();
        int x = Vis.transX(p.x);
        int y = Vis.transY(p.y);
        canvas.translate(x, y);
        canvas.scale(Vis.getZoomFactor(), Vis.getZoomFactor());

        float offset = size / 2 ;
        canvas.fillOval((int) (- offset),(int) (- offset), size, size);
        canvas.setTransform(t);
    }
    void drawArrow(Graphics2D canvas, int x1, int y1, int x2, int y2) {
        //Graphics2D g = (Graphics2D) g1.create();
        AffineTransform backup = canvas.getTransform();
        double dx = x2 - x1, dy = y2 - y1;
        double angle = Math.atan2(dy, dx);
        int len = (int) Math.sqrt(dx*dx + dy*dy);
        AffineTransform at = AffineTransform.getTranslateInstance(x1, y1);
        at.concatenate(AffineTransform.getRotateInstance(angle));
        canvas.transform(at);

        // Draw horizontal arrow starting in (0, 0)
        canvas.drawLine(0, 0, len, 0);
        canvas.fillPolygon(new int[]{len, len - ARR_SIZE, len - ARR_SIZE, len},
                new int[]{0, -ARR_SIZE, ARR_SIZE, 0}, 4);
        canvas.setTransform(backup);
    }
    public static VisLayer create(HighwayStorage storage) {
        PlansLayer layer = new PlansLayer(storage.getAgents(),storage.getPosCurr(),storage.getActions());
        return layer;
    }


}
