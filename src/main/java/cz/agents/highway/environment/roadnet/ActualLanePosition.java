package cz.agents.highway.environment.roadnet;

import javax.vecmath.Point2f;

/**
 * Created by david on 9/17/15.
 */
public class ActualLanePosition {
    private Lane lane;
    private int index;
    private Point2f innerPoint;

    public ActualLanePosition(Lane lane, int index, Point2f innerPoint) {
        this.lane = lane;
        this.index = index;
        this.innerPoint = innerPoint;
    }

    public Lane getLane() {
        return lane;
    }

    public int getIndex() {
        return index;
    }

    public Point2f getInnerPoint() {
        return innerPoint;
    }
}

