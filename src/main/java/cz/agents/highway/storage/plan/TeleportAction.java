package cz.agents.highway.storage.plan;

import javax.vecmath.Point3f;

/**
 * Created by david on 3.12.14.
 */
public class TeleportAction extends WPAction {
    public TeleportAction(int carId, double timeStamp,Point3f position,double speed) {
        super(carId, timeStamp,position,speed);
    }
}
