package cz.agents.agentdrive.simulator.lite.visualization;

import cz.agents.agentdrive.highway.environment.planning.euclid2d.Trajectory;
import cz.agents.agentdrive.highway.environment.planning.euclid3d.SpeedPoint;
import cz.agents.agentdrive.highway.environment.planning.euclid4d.Region;
import cz.agents.agentdrive.highway.environment.planning.euclid4d.region.MovingCircle;
import cz.agents.alite.vis.Vis;
import cz.agents.alite.vis.layer.AbstractLayer;
import cz.agents.alite.vis.layer.VisLayer;

import javax.vecmath.Point2d;
import java.awt.*;
import java.awt.geom.AffineTransform;

public class RegionsLayer extends AbstractLayer {
    public static int PRECISION = 10;


    private Region region;
    private Color edgeColor;
    private Color fillColor;
    private TimeParameter timeParameter;
    private int id;

    RegionsLayer() {
    }

    public RegionsLayer(Region region, TimeParameter timeParameter, Color edgeColor, Color fillColor, int id) {
        this.region = region;
        this.edgeColor = edgeColor;
        this.fillColor = fillColor;
        this.timeParameter = timeParameter;
        this.id = id;
    }

    @Override
    public void paint(Graphics2D canvas) {

        super.paint(canvas);


            if (region != null && region instanceof MovingCircle) {
                MovingCircle movingCirc = (MovingCircle) region;
                Trajectory traj = movingCirc.getTrajectory();

                assert(traj != null);

                SpeedPoint pos = traj.get((double)timeParameter.getTime()/(double)PRECISION);
                if (pos != null) {
                    Point2d projectedPoint = new Point2d(pos.x, pos.y);
                    if (fillColor != null) {
                        canvas.setColor(fillColor);
                        canvas.fillOval(
                                Vis.transX(projectedPoint.x
                                        - movingCirc.getRadius()),
                                Vis.transY(projectedPoint.y
                                        - movingCirc.getRadius()),
                                Vis.transH(movingCirc.getRadius()*2),
                                Vis.transW(movingCirc.getRadius()*2));
                    }

                    canvas.setColor(edgeColor);
                    canvas.drawOval(
                            Vis.transX(projectedPoint.x
                                    - movingCirc.getRadius()),
                            Vis.transY(projectedPoint.y
                                    - movingCirc.getRadius()),
                            Vis.transH(movingCirc.getRadius() * 2),
                            Vis.transW(movingCirc.getRadius() * 2));
                    canvas.setColor(Color.WHITE);
                    AffineTransform trans = canvas.getTransform();
                    canvas.translate(Vis.transX(projectedPoint.x), Vis.transY(projectedPoint.y));
                    canvas.scale(Vis.getZoomFactor()*0.2, Vis.getZoomFactor()*0.2);
                    canvas.drawString("" + id, -7, 5);
                    canvas.setTransform(trans);
                }
            }
    }

    public static VisLayer create(final Region region, TimeParameter timeParameter, final Color edgeColor, final Color fillColor, int id) {
        return new RegionsLayer(region, timeParameter, edgeColor, fillColor, id);
    }
}
