package cz.agents.highway.vis;


import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.highway.storage.RoadObject;

import javax.vecmath.Point3f;
import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Rectangle2D;
import java.util.Map;

/**
 * Created by martin on 14.7.14.
 */
public class RoadObjectLayer extends AbstractLayer {
    private static final double CAR_WIDTH = 2.0;
    private static final double CAR_AXE_LENGTH = 3.0;
    private static final double CAR_PADD = 1.0;
    private static final double REL_WHEEL_WIDTH = 0.333;
    private static final double REL_WHEEL_LEN = 0.25;

    private final Map<Integer, RoadObject> objects;
    


    RoadObjectLayer(Map<Integer, RoadObject> objects) {
        this.objects = objects;
    }

    @Override
    public void paint(Graphics2D canvas) {
        paintCars(canvas);
    }

    private void paintCars(Graphics2D graphics) {
        Graphics2D canvas = Vis.getCanvas();

        double wheelWidth = CAR_WIDTH*REL_WHEEL_WIDTH;
        for (Map.Entry<Integer,RoadObject> entry: objects.entrySet()) {
            // TODO: set different colors for agents
            int id = entry.getKey();
            RoadObject roadObject = entry.getValue();

            canvas.setColor(Color.BLUE);
            Point3f pos = roadObject.getPosition();
            double vehicleLen = CAR_AXE_LENGTH +CAR_PADD;
            double wheelLen = vehicleLen*REL_WHEEL_LEN;

            // Store the current transformation
            AffineTransform t = canvas.getTransform();

            // Transform the canvas according to the current zoom factor and axes translation
            canvas.translate(Vis.transX(pos.getX()), Vis.transY(pos.getY()));
            double vehicleRotation = Math.atan2(roadObject.getVelocity().getY(), roadObject.getVelocity().getX());
            canvas.scale(Vis.getZoomFactor(), Vis.getZoomFactor());
            canvas.setStroke(new BasicStroke(0.1f));
            canvas.drawLine(0, 0, (int) roadObject.getVelocity().x, (int) roadObject.getVelocity().y);
            canvas.setColor(Color.BLUE);
            canvas.rotate(vehicleRotation);

            // Draw vehicle body
            canvas.setColor(AgentColors.getColorForAgent(id));
            Rectangle2D body = new Rectangle2D.Double(-vehicleLen/2,  -CAR_WIDTH/2,
                    vehicleLen, CAR_WIDTH);
            canvas.fill(body);
            // Draw vehicle wheels
            canvas.setColor(Color.BLACK);
            // Rear
            canvas.fill(new Rectangle2D.Double(-vehicleLen / 2, -CAR_WIDTH / 2,
                    wheelLen, wheelWidth));
            canvas.fill(new Rectangle2D.Double(-vehicleLen / 2, CAR_WIDTH / 2 - wheelWidth,
                    wheelLen, wheelWidth));

            // Front
            Rectangle2D frontWheel = new Rectangle2D.Double(-wheelLen/2, -wheelWidth/2, wheelLen, wheelWidth);
            AffineTransform t2 = canvas.getTransform();
            canvas.translate(vehicleLen/2-wheelLen/2, -CAR_WIDTH/2+wheelWidth/2);
            //we do not know steering angle
            canvas.rotate(0);
            canvas.fill(frontWheel);
            canvas.setTransform(t2);

            canvas.translate(vehicleLen/2-wheelLen/2, CAR_WIDTH/2-wheelWidth/2);
          //  canvas.rotate(vehicle.getSteeringAngle());
            canvas.fill(frontWheel);

            canvas.setTransform(t);

        }
    }

    

    public static VisLayer create(Map<Integer,RoadObject> objects) {
        RoadObjectLayer layer = new RoadObjectLayer(objects);
        return layer;
    }
}
