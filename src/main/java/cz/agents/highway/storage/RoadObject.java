package cz.agents.highway.storage;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

public class RoadObject {

    private int id = -1;
    private double updateTime = -1;
    private Point3f position;
    private Vector3f velocity;
    private int lane = -1;

    public RoadObject(int id, double updateTime, int lane, Point3f position, Vector3f velocity) {
        this.id = id;
        this.updateTime = updateTime;
        this.lane = lane;
        this.position = position;
        this.velocity = velocity;

    }

    public RoadObject(int id) {
       this.id = id;
    }

    public int getId() {
        return id;
    }

    public double getUpdateTime() {
        return updateTime;
    }

    public int getLane() {
        return lane;
    }

    public Point3f getPosition() {
        return position;
    }

    public Vector3f getVelocity() {
        return velocity;
    }

    @Override
    public String toString() {
        return "RoadObject [id = " + id + ", updateTime=" + updateTime + ", lane=" + lane
                + ", pos=" + position + ", v=" + velocity + "]";
    }

}
