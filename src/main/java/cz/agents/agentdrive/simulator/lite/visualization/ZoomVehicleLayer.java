/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package cz.agents.agentdrive.simulator.lite.visualization;

import cz.agents.agentdrive.highway.environment.roadnet.Lane;
import cz.agents.agentdrive.highway.environment.roadnet.Network;
import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.simulator.lite.storage.VehicleStorage;
import cz.agents.agentdrive.simulator.lite.storage.vehicle.Vehicle;
import cz.agents.alite.configurator.Configurator;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.CommonLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;

import javax.vecmath.Point2d;
import javax.vecmath.Point2f;
import javax.vecmath.Vector2d;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.geom.AffineTransform;
import java.awt.geom.Line2D;
import java.awt.geom.Rectangle2D;
import java.util.LinkedList;

/**
 *
 * @author ondra
 */
public class ZoomVehicleLayer extends CommonLayer{
    private final double MY_ZOOM = 3;

    private final int PLACE_FOR_TEXT = 20;
    private final int HEIGHT = 250;
    private final int RADAR_HEIGHT = (int) (((HEIGHT)/2 -10));
    private final int WIDTH =250;
    private final int RADAR_WIDTH = (int) ((WIDTH /2 -10));
    
    private final int WIDTH_LINE = 4;
    private final int TEXT_OFFSET = 10;
    
    private final double SEARCHING_DIST = HEIGHT/4;
    
    
    private final int VELOCITY_WIDTH = (int) (1 * MY_ZOOM);
    private final int VELOCITY_LENGTH = (int)(1 * MY_ZOOM);
    private final int CAR_WIDTH = (int) (2 * MY_ZOOM);
    private final int CAR_LENGTH = (int) (4 * MY_ZOOM);
    private final int WHEEL_WIDTH = (int) (0.75 * MY_ZOOM);
    private final int WHEEL_LENGTH = (int) (1 * MY_ZOOM);
    
    private final int LANE_WIDTH = (int) (4 * MY_ZOOM);
    
    
    private final VehicleStorage storage;
    private int indexOfCar;
    private Vehicle mainVehicle = null;
    private Rectangle2D drawingRectangle;
    private Network net;
    
    private boolean settingRotation = false;
    
    protected ZoomVehicleLayer(VehicleStorage storage){
        this.storage = storage;
        XMLReader reader = new XMLReader(Configurator.getParamString("simulator.net.folder", "nets/junction-big/"));
        net = reader.getNetwork();
    }
    
    @Override
    public void paint(Graphics2D canvas) {
        Dimension dim = Vis.getDrawingDimension();
        
        int posX = (int)(dim.getWidth() - WIDTH);
        int posY = (int)(dim.getHeight() - HEIGHT - PLACE_FOR_TEXT);
        
        canvas.setColor(Color.LIGHT_GRAY);
        canvas.fillRect(posX, posY, WIDTH, HEIGHT + PLACE_FOR_TEXT);
        canvas.setColor(Color.BLACK);
        canvas.setStroke(new BasicStroke(WIDTH_LINE, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        canvas.drawRect(posX - WIDTH_LINE/2, posY - WIDTH_LINE/2, WIDTH + WIDTH_LINE, (HEIGHT+PLACE_FOR_TEXT) + WIDTH_LINE);
        

        
        if (mainVehicle != null){
            canvas.drawString("Index of main vehicle: "+mainVehicle.getId(), posX + TEXT_OFFSET, posY + TEXT_OFFSET);
            AffineTransform centerOfToolWindow = canvas.getTransform();
            int centerX = (int)(dim.getWidth() -WIDTH/2);
            int centerY = (int)(dim.getHeight() -(HEIGHT)/2);

            canvas.translate(centerX, centerY);

            drawingRectangle = new Rectangle(-RADAR_WIDTH /2, -RADAR_HEIGHT/2, WIDTH, HEIGHT);

            if(settingRotation){
                double vehicleRotation = Math.atan2(mainVehicle.getHeading().getY(), mainVehicle.getHeading().getX());
                canvas.rotate(-vehicleRotation);
            }

            canvas.setColor(Color.DARK_GRAY);
            int size = LANE_WIDTH;
            canvas.setStroke(new BasicStroke(size, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            for (Lane lane : net.getLanes().values()) {
                Point2f prev = lane.getShape().get(0);
                Vector2d vecPrev = vector(mainVehicle, prev);
                for (Point2f point : lane.getShape()) {
                    Vector2d vecPoint = vector(mainVehicle, point);
                    paintSCALELine(canvas,vecPrev,vecPoint);
                    vecPrev = vecPoint;

                }
                for (Lane outgoingLane : lane.getOutgoingLanes()) {
                    Point2f point = outgoingLane.getShape().get(0);
                    Vector2d vecPoint = vector(mainVehicle, point);
                    paintLine(canvas,vecPrev,vecPoint);
                    vecPrev = vecPoint;
                }
            }

            drawVehicle(canvas, mainVehicle);
            for (Vehicle v : storage.getAllVehicles()){
                if (!v.equals(mainVehicle)){
                    Vector2d vector= vector(mainVehicle, v);
                    if (isIn(vector)){
                        AffineTransform t2 = canvas.getTransform();
                        canvas.translate(vector.x * MY_ZOOM, vector.y * MY_ZOOM);
                        drawVehicle(canvas, v);
                        canvas.setTransform(t2);
                    }
                }

            }

            canvas.setTransform(centerOfToolWindow);
        }else{
            canvas.drawString("START THIS PLUGIN", posX + TEXT_OFFSET, posY + TEXT_OFFSET);
            canvas.drawString("BY PRESSING 'p'", posX + TEXT_OFFSET, posY + TEXT_OFFSET + 20);
        }
        
        
    }
    
    
    
    public static VisLayer create(VehicleStorage storage) {
        KeyToggleLayer toggle = KeyToggleLayer.create("o");
        toggle.addSubLayer(new ZoomVehicleLayer(storage));
        return toggle;
    }
    
    @Override
    public void init(Vis vis) {
        super.init(vis);

        vis.addKeyListener(new KeyListener() {

            public void keyTyped(KeyEvent e) {
            }

            public void keyReleased(KeyEvent e) {
            }

            public void keyPressed(KeyEvent e) {
                if (e.getKeyChar() == 'p') {
                    indexOfCar = (int) ((indexOfCar + 1) % storage.getVehicles().size());
                    LinkedList<Vehicle> list = new LinkedList<>(storage.getVehicles());
                    if (list.isEmpty()){
                        mainVehicle = null;
                    }else{
                        mainVehicle = list.get(indexOfCar);
                    }
                }else if(e.getKeyChar() == 'l'){
                    settingRotation = !settingRotation;
                } 
                
            }
        });
    }
    
    @Override
    public String getLayerDescription() {
        String description = "[Vis Zoom] The layer shows zoom deatil at vehicle info in lower right corner, you can change selected vehicle by pressing 'p' to hide window press 'o' and to chage rotation press 'l'.";
        return buildLayersDescription(description);
    }
    

    
    private Vector2d vector(Vehicle main, Vehicle another){
        return new Vector2d(another.getPosition().x - main.getPosition().x,
                another.getPosition().y - main.getPosition().y);
    }
    private Vector2d vector(Vehicle main, Point2f p){
        return new Vector2d(p.x - main.getPosition().x, p.y - main.getPosition().y);
    }
    
    private boolean isIn(Vector2d v){
        return (-v.x * MY_ZOOM < RADAR_WIDTH && -v.y * MY_ZOOM < RADAR_HEIGHT);
    }
    private boolean isIn(Point2d p){
        return (Math.abs(p.x) * MY_ZOOM < RADAR_WIDTH && Math.abs(p.y) * MY_ZOOM < RADAR_HEIGHT);
    }
    
    
    private void drawVehicle(Graphics2D canvas, Vehicle vehicle){
        AffineTransform drawingCarTrans = canvas.getTransform();
        canvas.setColor(Color.BLUE);
        canvas.setStroke(new BasicStroke(VELOCITY_WIDTH));
        Line2D line = new Line2D.Double(0, 0,
                vehicle.getHeading().x*vehicle.getVelocity() * VELOCITY_LENGTH,
                vehicle.getHeading().y*vehicle.getVelocity() * VELOCITY_LENGTH);
        canvas.draw(line);

        double vehicleRotation = Math.atan2(vehicle.getHeading().getY(), vehicle.getHeading().getX());
        canvas.rotate(vehicleRotation);

        // Draw vehicle body
        canvas.setColor(Color.RED);
        Rectangle2D body = new Rectangle2D.Double(-CAR_LENGTH /2,  -CAR_WIDTH/2,
                CAR_LENGTH, CAR_WIDTH);
        canvas.fill(body);
        // Draw vehicle wheels
        canvas.setColor(Color.BLACK);
        canvas.fill(new Rectangle2D.Double(-CAR_LENGTH / 2, -CAR_WIDTH / 2,
                WHEEL_LENGTH, WHEEL_WIDTH));
        canvas.fill(new Rectangle2D.Double(-CAR_LENGTH / 2, CAR_WIDTH / 2 - WHEEL_WIDTH,
                WHEEL_LENGTH, WHEEL_WIDTH));

        // Front
        Rectangle2D frontWheel = new Rectangle2D.Double(-WHEEL_LENGTH /2, -WHEEL_WIDTH/2, WHEEL_LENGTH, WHEEL_WIDTH);
        AffineTransform t2 = canvas.getTransform();
        canvas.translate(CAR_LENGTH /2- WHEEL_LENGTH /2, -CAR_WIDTH/2+WHEEL_WIDTH/2);
        canvas.rotate(vehicle.getSteeringAngle());
        canvas.fill(frontWheel);
        canvas.setTransform(t2);

        canvas.translate(CAR_LENGTH / 2 - WHEEL_LENGTH / 2, CAR_WIDTH / 2 - WHEEL_WIDTH / 2);
        canvas.rotate(vehicle.getSteeringAngle());
        canvas.fill(frontWheel);
        canvas.setTransform(drawingCarTrans);
    }
    
    void paintSCALELine(Graphics2D canvas, Vector2d v1, Vector2d v2){
        Point2d p1 = new Point2d(v1.x * MY_ZOOM, v1.y * MY_ZOOM);
        Point2d p2 = new Point2d(v2.x * MY_ZOOM, v2.y * MY_ZOOM);
        Line2D line2d = new Line2D.Double(p1.x, p1.y, p2.x, p2.y);
        paintLine(canvas, v1,v2);
        if (isIn(v1) || isIn(v2)) {
            if (!isIn(v1)){
                Vector2d vec = new Vector2d(v1.x - v2.x, v1.y - v2.y);
                while(!isIn(vec)){
                    vec.x = vec.x*0.9;
                    vec.y = vec.y*0.9;
                }
                AffineTransform t2 = canvas.getTransform();
                canvas.translate((int)(p2.x), (int)(p2.y));
                line2d = new Line2D.Double(0, 0, vec.x, vec.y);
                canvas.draw(line2d);
                canvas.setTransform(t2);
            }

            if (!isIn(v2)){
                Vector2d vec = new Vector2d(v2.x - v1.x, v2.y - v1.y);
                while(!isIn(vec)){
                    vec.x = vec.x*0.9;
                    vec.y = vec.y*0.9;
                }
                AffineTransform t2 = canvas.getTransform();
                canvas.translate((int)(p1.x), (int)(p1.y));
                line2d = new Line2D.Double(0, 0, vec.x, vec.y);
                canvas.draw(line2d);
                canvas.setTransform(t2);
            }

        }
    }

    void paintLine(Graphics2D canvas, Vector2d v1, Vector2d v2){
        if (isIn(v1) && isIn(v2)){
            Point2d p1 = new Point2d(v1.x * MY_ZOOM, v1.y * MY_ZOOM);
            Point2d p2 = new Point2d(v2.x * MY_ZOOM, v2.y * MY_ZOOM);
            Line2D line2d = new Line2D.Double(p1.x, p1.y, p2.x, p2.y);
            canvas.draw(line2d);
        }


    }

}
