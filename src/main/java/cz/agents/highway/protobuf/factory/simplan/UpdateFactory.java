package cz.agents.highway.protobuf.factory.simplan;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import com.google.protobuf.InvalidProtocolBufferException;

import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.highway.protobuf.generated.simplan.MessageContainer.Message;
import cz.agents.highway.protobuf.generated.simplan.UpdateMessage.Update;
import cz.agents.highway.protobuf.generated.simplan.UpdateMessage.Update.Builder;
import cz.agents.highway.protobuf.generated.simplan.UpdateMessage.Vehicle_update;
import cz.agents.highway.protobuf.generated.simplan.VectorProto.Vector;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.RoadObject;

public class UpdateFactory implements FactoryInterface<RadarData, Message> {

    public Message encode(Object object) {
        double updateTime = Double.MAX_VALUE;

        Builder updateBuilder = Update.newBuilder();
        if (object.getClass().equals(this.getObjectClass())) {
            RadarData radarData = (RadarData) object;
            for (RoadObject car : radarData.getCars()) {
                cz.agents.highway.protobuf.generated.simplan.UpdateMessage.Vehicle_update.Builder veh_builder = Vehicle_update
                        .newBuilder();

                Point3f pos = car.getPosition();
                Vector3f v = car.getVelocity();

                Vector position = Vector.newBuilder().setX(pos.x).setY(pos.y).setZ(pos.z).build();
                Vector velocity = Vector.newBuilder().setX(v.x).setY(v.y).setZ(v.z).build();

                veh_builder.setVehicleId(car.getId()).setLane(car.getLaneIndex()).setVelocity(velocity);
                veh_builder.setAcceleration(0).setPosition(position);

                // updateTime
                updateTime = (double) Math.min(updateTime, car.getUpdateTime());
                updateBuilder.addUpdates(veh_builder.build());
            }

        }

        return Message.newBuilder().setUpdate(updateBuilder.setUpdateTime(updateTime).build())
                .build();
    }

    public RadarData decode(Message pm) throws InvalidProtocolBufferException {
        if (!pm.hasUpdate()) {
            return null;
        }

        Update update = pm.getUpdate();
        RadarData carUpdates = new RadarData();
        for (Vehicle_update vu : update.getUpdatesList()) {
            double time = update.getUpdateTime();
            Vector p = vu.getPosition();
            Vector v = vu.getVelocity();
            Point3f pos = new Point3f((float) p.getX(), (float) p.getY(), (float) p.getZ());
            Vector3f vel = new Vector3f((float) v.getX(), (float) v.getY(), (float) v.getZ());
            carUpdates.add(new RoadObject(vu.getVehicleId(), time, vu.getLane(), pos, vel));
        }

        return carUpdates;
    }

    public Class<RadarData> getObjectClass() {
        return RadarData.class;
    }

}
