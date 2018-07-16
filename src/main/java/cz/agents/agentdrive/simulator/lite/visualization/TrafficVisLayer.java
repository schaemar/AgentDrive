package cz.agents.agentdrive.simulator.lite.visualization;

import cz.agents.agentdrive.simulator.lite.storage.VehicleStorage;
import cz.agents.agentdrive.simulator.lite.storage.vehicle.Ghost;
import cz.agents.agentdrive.simulator.lite.storage.vehicle.Vehicle;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;

import java.awt.*;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import javax.vecmath.Point3f;

/**
 * Created by wmatex on 8.7.14.
 */
public class TrafficVisLayer extends AbstractLayer {
    private static final double CAR_WIDTH = 2.0;
    private static final double CAR_PADD = 1.0;
    private static final double REL_WHEEL_WIDTH = 0.333;
    private static final double REL_WHEEL_LEN = 0.25;

    private final VehicleStorage storage;
    


    TrafficVisLayer(VehicleStorage storage) {
        this.storage = storage;
    }

    @Override
    public void paint(Graphics2D canvas) {
        paintCars(canvas);
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

    private void paintCars(Graphics2D graphics) {
        Graphics2D canvas = Vis.getCanvas();

        double wheelWidth = CAR_WIDTH*REL_WHEEL_WIDTH;
        for (Vehicle vehicle: storage.getVehicles()) {
            // TODO: set different colors for agents
            canvas.setColor(Color.BLUE);
            Point3f pos = vehicle.getPosition();
            double vehicleLen = vehicle.getAxeLength()+CAR_PADD;
            double wheelLen = vehicleLen*REL_WHEEL_LEN;

            // Store the current transformation
            AffineTransform t = canvas.getTransform();

            // Tranform the canvas according to the current zoom factor and axes translation
            canvas.translate(Vis.transX(pos.getX()), Vis.transY(pos.getY()));
            double vehicleRotation = Math.atan2(vehicle.getHeading().getY(), vehicle.getHeading().getX());
            canvas.scale(Vis.getZoomFactor(), Vis.getZoomFactor());
            canvas.setStroke(new BasicStroke(1));
            Line2D line = new Line2D.Double(0, 0,
                    vehicle.getHeading().x*vehicle.getVelocity(),  vehicle.getHeading().y*vehicle.getVelocity());
            canvas.draw(line);
            
            //Draw vehicle velocity
            canvas.setColor(Color.BLUE);
            canvas.rotate(vehicleRotation);

            // Draw vehicle body
            canvas.setColor(Color.RED);
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
            canvas.rotate(vehicle.getSteeringAngle());
            canvas.fill(frontWheel);
            canvas.setTransform(t2);

            canvas.translate(vehicleLen/2-wheelLen/2, CAR_WIDTH/2-wheelWidth/2);
            canvas.rotate(vehicle.getSteeringAngle());
            canvas.fill(frontWheel);

            canvas.setTransform(t);
            
            for(Point3f wayPoint : vehicle.getPlanController().getAllNextWayPoints()){
                paintCircle(canvas, wayPoint, 1);
            }

        }

        for (Ghost ghost: storage.getGhosts()) {
            canvas.setColor(Color.GRAY);
            canvas.fillRect(Vis.transX(ghost.getPosition().x), Vis.transY(ghost.getPosition().y),
                    Vis.transH(CAR_WIDTH), Vis.transW(ghost.getAxeLength()+CAR_PADD));
        }
    }

    

    public static VisLayer create(VehicleStorage storage) {
        TrafficVisLayer layer = new TrafficVisLayer(storage);
        return layer;
    }
}
