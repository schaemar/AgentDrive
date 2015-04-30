package cz.agents.highway.vis;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.event.InputEvent;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.text.MessageFormat;

import cz.agents.alite.common.event.EventProcessorEventType;
import cz.agents.alite.simulation.Simulation;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;
import cz.agents.alite.vis.layer.common.HelpLayer;
import cz.agents.alite.vis.layer.toggle.KeyToggleLayer;
import cz.agents.highway.environment.HighwayEnvironment;

/**
 * The layer shows the status of the simulation and controls it through various
 * key bindings.
 *
 * The information shown, tells the user, the current simulation speed (ratio of
 * the real time and simulation time) and the state of the simulation.
 *
 * The simulation speed ratio can be controlled by '+' and '-' keys. And
 * additionally, Ctrl+'*' sets the fastest possible speed (infinite ratio), and
 * '*' pressed sets the ratio to its default value.
 *
 * All the possible key strokes are described in the internal help showed by the
 * {@link HelpLayer}.
 *
 *
 * @author Antonin Komenda
 */
public class SimulationControlLayer extends AbstractLayer {

    private final Simulation simulation;
    private HighwayEnvironment highwayEnvironment;

    SimulationControlLayer(Simulation simulation,HighwayEnvironment highwayEnvironment) {
        this.simulation = simulation;
        this.highwayEnvironment = highwayEnvironment;
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
                if (e.getKeyChar() == '+') {
                    simulation.setSimulationSpeed(simulation.getSimulationSpeed() * 0.9);
                } else if (e.getKeyChar() == '-') {
                    simulation.setSimulationSpeed(simulation.getSimulationSpeed() * 1.1);
                } else if (e.getKeyChar() == '*') {
                    if ((e.getModifiers() & (InputEvent.CTRL_MASK | InputEvent.CTRL_DOWN_MASK)) != 0) {
                        simulation.setSimulationSpeed(0);
                    } else {
                        simulation.setSimulationSpeed(0.2);
                    }
                } else if (e.getKeyChar() == ' ') {
                    if (simulation.isRunning()) {
                        simulation.setRunning(false);
                    } else {
                        simulation.setRunning(true);
                    }
                }
                else if (e.getKeyChar() == 'x')
                {
                   simulation.addEvent(EventProcessorEventType.STOP, null, null, null);
                }
            }
        });
    }

    @Override
    public void paint(Graphics2D canvas) {
        StringBuilder label = new StringBuilder();
        label.append("TIME: ");
        label.append(simulation.getCurrentTime() / 1000.0);
        label.append(" ");
        label.append(System.getProperty("line.separator"));
        if (simulation.isFinished()) {
            label.append("TIME in seconds: ");
            label.append((highwayEnvironment.getStorage().getENDTIME() - highwayEnvironment.getStorage().getSTARTTIME()) / 1000.0);
            label.append(" ");
            label.append("(FINISHED)");
        } else {
            label.append("TIME in seconds: ");
            label.append((System.currentTimeMillis() - highwayEnvironment.getStorage().getSTARTTIME()) / 1000.0);
            label.append(" ");
            if (simulation.getCurrentTime() == 0) {
                label.append("(INITIALIZING)");

                canvas.setColor(new Color(0, 0, 0, 200));
                canvas.fillRect(200, 400, Vis.getDrawingDimension().width - 400, Vis.getDrawingDimension().height - 800);

                Font oldFont = canvas.getFont();
                canvas.setFont(new Font("Arial", 0, 20));
                canvas.setColor(Color.WHITE);
                canvas.drawString("INITIALIZING...", Vis.getDrawingDimension().width / 2 - 60 , Vis.getDrawingDimension().height / 2 + 7);
                canvas.setFont(oldFont);
            } else {
                if (simulation.isRunning()) {
                        label.append("(");
                        label.append(MessageFormat.format("{0,number,#.##}", 1/simulation.getSimulationSpeed()/5.0));
                        label.append("x)");
                } else {
                    label.append("(PAUSED)");
                }
            }
        }

        canvas.setColor(Color.BLUE);
        canvas.drawString(label.toString(), 15, 20);
    }

    @Override
    public String getLayerDescription() {
        String description = "[Simulation Control] Layer controls the simulation and shows simulation time and speed,\n" +
                "by pressing '<space>', the simulation can be paused and unpaused,\n" +
                "by pressing '+'/'-', the simulation can be speed up and slow down,\n" +
                "by pressing '*', the speed of simulation is set to default value (1x),\n" +
                "by pressing 'q', simulation ends,\n" +
                "by pressing Ctrl+'*', the speed of simulation is set to fastest possible speed ().";
        return buildLayersDescription(description);
    }

    public static VisLayer create(Simulation simulation,HighwayEnvironment highwayEnvironment) {
        VisLayer simulationControl = new SimulationControlLayer(simulation,highwayEnvironment);

        KeyToggleLayer toggle = KeyToggleLayer.create("s");
        toggle.addSubLayer(simulationControl);
        toggle.setHelpOverrideString(simulationControl.getLayerDescription() + "\n" +
                "By pressing 's', the simulation info can be turned off and on.");

        return toggle;
    }

}
