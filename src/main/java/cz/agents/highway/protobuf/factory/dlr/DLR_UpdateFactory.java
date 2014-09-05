package cz.agents.highway.protobuf.factory.dlr;

import javax.vecmath.Point3f;
import javax.vecmath.Vector3f;

import com.google.protobuf.InvalidProtocolBufferException;

import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.highway.protobuf.generated.dlr.DLR_MessageContainer.Message;
import cz.agents.highway.protobuf.generated.dlr.DLR_UpdateMessage.Update;
import cz.agents.highway.protobuf.generated.dlr.DLR_UpdateMessage.Update.Builder;
import cz.agents.highway.protobuf.generated.dlr.DLR_UpdateMessage.Vehicle_update;
import cz.agents.highway.storage.RadarData;
import cz.agents.highway.storage.RoadObject;



public class DLR_UpdateFactory implements FactoryInterface<RadarData, Message> {

    public Message encode(Object object) {
        float updateTime = Float.MAX_VALUE;

        Builder updateBuilder = Update.newBuilder();
        if (object.getClass().equals(this.getObjectClass())) {
            RadarData radarData = (RadarData) object;
            for (RoadObject car : radarData.getCars()) {
                cz.agents.highway.protobuf.generated.dlr.DLR_UpdateMessage.Vehicle_update.Builder veh_builder = Vehicle_update
                        .newBuilder();

                Point3f pos = car.getPosition();
                Vector3f v = car.getVelocity();

                veh_builder.setPosX(pos.x).setPosY(pos.y).setPosZ(pos.z);

                //TODO hacked lane = lane + 1 to meet dlr lane indexing - check DLR_PlansFactory.java too
                veh_builder.setVehicleId(car.getId()).setLane(car.getLaneIndex()+1).setVelocityX(v.x).setVelocityY(v.y);

                // updateTime
                updateTime = (float) Math.min(updateTime, car.getUpdateTime());
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
            
            
            Point3f pos = new Point3f((float) vu.getPosX(), (float) vu.getPosY(), (float) vu.getPosZ());
            Vector3f vel = new Vector3f((float) vu.getVelocityX(), (float) vu.getVelocityY(), 0.0f);
            carUpdates.add(new RoadObject(vu.getVehicleId(), time, vu.getLane(), pos, vel));
        }

        return carUpdates;
    }

    public Class<RadarData> getObjectClass() {
        return RadarData.class;
    }


}
