package cz.agents.agentdrive.simulator.lite.visualization;

import cz.agents.agentdrive.highway.agent.Agent;
import cz.agents.agentdrive.highway.agent.SDAgent;
import cz.agents.agentdrive.highway.environment.roadnet.Lane;
import cz.agents.agentdrive.highway.environment.roadnet.Network;
import cz.agents.agentdrive.highway.environment.roadnet.XMLReader;
import cz.agents.agentdrive.highway.maneuver.CarManeuver;
import cz.agents.agentdrive.highway.storage.HighwayStorage;
import cz.agents.agentdrive.highway.storage.RoadObject;
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

public class StateSpaceVehicleLayer extends CommonLayer {

    private final int PLACE_FOR_TEXT = 20;
    private final int HEIGHT = 250;
    private final int WIDTH =250;

    private final int WIDTH_LINE = 4;
    private final int TEXT_OFFSET = 10;

    private int indexOfCar;
    private SDAgent mainAgent = null;
    private Integer mainAgentID = -1;
    private RoadObject mainRoadObject = null;
    private Rectangle2D drawingRectangle;
    private Network net;

    private boolean settingRotation = false;
    private HighwayStorage highwayStorage;

    protected StateSpaceVehicleLayer(HighwayStorage highwayStorage){
        this.highwayStorage = highwayStorage;
        XMLReader reader = new XMLReader(Configurator.getParamString("simulator.net.folder", "nets/junction-big/"));
        net = reader.getNetwork();
    }

    @Override
    public void paint(Graphics2D canvas) {
        mainAgent = (SDAgent)highwayStorage.getAgents().get(mainAgentID);
        mainRoadObject = highwayStorage.getPosCurr().get(mainAgentID);
        Dimension dim = Vis.getDrawingDimension();

        int posX = (int)(dim.getWidth() - WIDTH);
        int posY = (int)(dim.getHeight() - HEIGHT - PLACE_FOR_TEXT);

        canvas.setColor(Color.LIGHT_GRAY);
        canvas.fillRect(posX, posY, WIDTH, HEIGHT + PLACE_FOR_TEXT);
        canvas.setColor(Color.BLACK);
        canvas.setStroke(new BasicStroke(WIDTH_LINE, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        canvas.drawRect(posX - WIDTH_LINE/2, posY - WIDTH_LINE/2, WIDTH + WIDTH_LINE, (HEIGHT+PLACE_FOR_TEXT) + WIDTH_LINE);



        if (mainAgent != null){
            canvas.drawString("Index of main vehicle: "+ mainAgent.getName(), posX + TEXT_OFFSET, posY + TEXT_OFFSET);
            AffineTransform centerOfToolWindow = canvas.getTransform();
            int centerX = (int)(dim.getWidth() -WIDTH);
            int centerY = (int)(dim.getHeight() -(HEIGHT));

            canvas.translate(centerX, centerY);
            canvas.setColor(Color.green);

            canvas.drawRect(0, 0, WIDTH/3, HEIGHT/3); // Ahead left
            canvas.drawRect(WIDTH/3, 0, WIDTH/3, HEIGHT/3); // Ahead
            canvas.drawRect(2*WIDTH/3, 0, WIDTH/3, HEIGHT/3); // Ahead right

            canvas.drawRect(0, HEIGHT/3, WIDTH/3, 3*HEIGHT/3); // left
            canvas.drawRect(WIDTH/3, HEIGHT/3, WIDTH/3, HEIGHT/3); // center
            canvas.drawRect(2*WIDTH/3, HEIGHT/3, WIDTH/3, 3*HEIGHT/3); // right
            canvas.setColor(Color.BLACK);
            int y = canvas.getFontMetrics().getHeight(), x = 0;
            if (mainAgent.getHighwaySituation().getCarLeftAheadMan() != null){
                drawManeuver(mainAgent.getHighwaySituation().getCarLeftAheadMan(),canvas, x, y);
            }
            x = WIDTH/3;
            y = canvas.getFontMetrics().getHeight();

            if (mainAgent.getHighwaySituation().getCarAheadMan() != null){
                drawManeuver(mainAgent.getHighwaySituation().getCarAheadMan(),canvas, x, y);
            }
            x = 2*WIDTH/3;
            y =canvas.getFontMetrics().getHeight();
            if (mainAgent.getHighwaySituation().getCarRightAheadMan() != null){
                drawManeuver(mainAgent.getHighwaySituation().getCarRightAheadMan(),canvas, x, y);
            }
            x=WIDTH/3;
            y=HEIGHT/3 + canvas.getFontMetrics().getHeight();
            if (mainAgent.getCurrentManeuver() != null){
                drawManeuver(mainAgent.getCurrentManeuver(), canvas, x, y);
            }
            x = 0;
            y = HEIGHT / 3 + canvas.getFontMetrics().getHeight();
            if (mainAgent.getHighwaySituation().getCarLeftMan() != null){
                drawManeuver(mainAgent.getHighwaySituation().getCarLeftMan(),canvas, x, y);
            }
            x = 3*WIDTH/3;
            y = HEIGHT /3 + canvas.getFontMetrics().getHeight();
            if (mainAgent.getHighwaySituation().getCarRightMan() != null){
                drawManeuver(mainAgent.getHighwaySituation().getCarRightMan(),canvas, x, y);
            }
            //canvas.drawString(Double.toString());
            canvas.drawString("Ahead", WIDTH/3, 0);
            canvas.drawString("Ahead right", 2*WIDTH/3, 0);
            canvas.drawString("left", 0, HEIGHT/3);
            canvas.drawString("right", 2*WIDTH/3, HEIGHT/3);
            canvas.setColor(AgentColors.getColorForAgent(mainAgentID));
            canvas.drawString("center", WIDTH/3, HEIGHT/3);

            canvas.setTransform(centerOfToolWindow);
        }else{
            canvas.drawString("START THIS PLUGIN", posX + TEXT_OFFSET, posY + TEXT_OFFSET);
            canvas.drawString("BY PRESSING 't'", posX + TEXT_OFFSET, posY + TEXT_OFFSET + 20);
        }

    }

    public static VisLayer create(HighwayStorage highwayStorage) {
        KeyToggleLayer toggle = KeyToggleLayer.create("u");
        toggle.addSubLayer(new StateSpaceVehicleLayer(highwayStorage));
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
                if (e.getKeyChar() == 't') {

                    indexOfCar = (int) ((indexOfCar + 1) % highwayStorage.getAgents().size());
                    LinkedList<Integer> list = new LinkedList<>(highwayStorage.getAgents().keySet());
                    if (list.isEmpty()){
                        mainAgent = null;
                    }else{

                        mainAgentID = list.get(indexOfCar);
                        mainAgent = (SDAgent)highwayStorage.getAgents().get(mainAgentID);
                        mainRoadObject = highwayStorage.getPosCurr().get(mainAgentID);
                    }
                }else if(e.getKeyChar() == 'l'){
                    settingRotation = !settingRotation;
                }

            }
        });
    }

    @Override
    public String getLayerDescription() {
        String description = "[Vis Zoom] The layer shows zoom detail at vehicle info in lower right corner, you can change selected vehicle by pressing 't' to hide window press 'u' and to chage rotation press 'l'.";
        return buildLayersDescription(description);
    }

    public void drawManeuver(CarManeuver carManeuver, Graphics2D g2d, int x, int y){
        g2d.drawString(Double.toString(carManeuver.getLaneOut()), x, y);
        g2d.drawString(Boolean.toString(carManeuver.isSafeManeuver()), x, y  + g2d.getFontMetrics().getHeight());
        g2d.drawString(String.format("%.3f", carManeuver.getPositionIn()), x, y + 2*g2d.getFontMetrics().getHeight());
        g2d.drawString(String.format("%.3f", carManeuver.getPositionOut()), x, y + 3*g2d.getFontMetrics().getHeight());
    }



}
