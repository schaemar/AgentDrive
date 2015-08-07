package cz.agents.highway.simpleMessage;

import cz.agents.highway.storage.RoadObject;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;


/**
 * Created by michal on 8.2.15.
 */
public class ComMessage {
    private MesType type;
    private int senderID;
    private int receiverID;
    private String contain;
    private RoadObject object;
    private int convoyPosition;
    private double referenceVelocity;
    private double headSensor;

    // Basic message
    public ComMessage(int senderID, int recievedID, String contain) {
        this.senderID = senderID;
        this.receiverID = recievedID;
        this.contain = contain;
    }
    // Broadcast message
    public ComMessage(int senderID, String contain) {
        this.senderID = senderID;
        this.contain = contain;
    }
    // Whole message


    public ComMessage(MesType type, int senderID, RoadObject object) {
        this.type = type;
        this.senderID = senderID;
        this.object = object;
    }

    public ComMessage(MesType type, int senderID, int receiverID, String contain, RoadObject object, int convoyPosition, double referenceVelocity, double headSensor) {
        this.type = type;
        this.senderID = senderID;
        this.receiverID = receiverID;
        this.contain = contain;
        this.object = object;
        this.convoyPosition = convoyPosition;
        this.referenceVelocity = referenceVelocity;
        this.headSensor = headSensor;
    }

    @Override
    public boolean equals(Object o) {
        if (o == null) return false;
        if (this == o) return true;
        if (!(o instanceof ComMessage)) return false;

        ComMessage that = (ComMessage) o;

        if (convoyPosition != that.convoyPosition) return false;
        if (Double.compare(that.headSensor, headSensor) != 0) return false;
        if (receiverID != that.receiverID) return false;
        if (Double.compare(that.referenceVelocity, referenceVelocity) != 0) return false;
        if (senderID != that.senderID) return false;
        if (contain != null ? !contain.equals(that.contain) : that.contain != null) return false;
        if (!object.equals(that.object)) return false;
        if (type != that.type) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        result = type.hashCode();
        result = 31 * result + senderID;
        result = 31 * result + receiverID;
        result = 31 * result + (contain != null ? contain.hashCode() : 0);
        result = 31 * result + object.hashCode();
        result = 31 * result + convoyPosition;
        temp = Double.doubleToLongBits(referenceVelocity);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(headSensor);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    public int getSenderID() {
        return senderID;
    }

    public void setSenderID(int senderID) {
        this.senderID = senderID;
    }

    public int getReceiverID() {
        return receiverID;
    }

    public void setReceiverID(int receiverID) {
        this.receiverID = receiverID;
    }

    public String getContain() {
        return contain;
    }

    public void setContain(String contain) {
        this.contain = contain;
    }

    public RoadObject getObject() {
        return object;
    }

    public void setObject(RoadObject object) {
        this.object = object;
    }

    public int getConvoyPosition() {
        return convoyPosition;
    }

    public void setConvoyPosition(int convoyPosition) {
        this.convoyPosition = convoyPosition;
    }

    public double getReferenceVelocity() {
        return referenceVelocity;
    }

    public void setReferenceVelocity(double referenceVelocity) {
        this.referenceVelocity = referenceVelocity;
    }

    public MesType getType() {
        return type;
    }

    public void setType(MesType type) {
        this.type = type;
    }
}
