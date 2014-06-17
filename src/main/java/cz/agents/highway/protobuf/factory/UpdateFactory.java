package cz.agents.highway.protobuf.factory;

import javax.vecmath.Point2d;

import com.google.protobuf.InvalidProtocolBufferException;

import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.highway.protobuf.generated.MessageContainer.Message;
import cz.agents.highway.protobuf.generated.UpdateMessage.Update;
import cz.agents.highway.protobuf.generated.UpdateMessage.Vehicle_update;
import cz.agents.highway.storage.CarIn;
import cz.agents.highway.storage.CarsIn;



public class UpdateFactory implements FactoryInterface<CarsIn, Message> {

    // private static final double ROAD_OFFSET = -10;//-10.;//2.5;
    public Message encode(Object object) {
        float updateTime = Float.MAX_VALUE;

        cz.agents.highway.protobuf.generated.UpdateMessage.Update.Builder updateBuilder = Update.newBuilder();
        if(object.getClass().equals(this.getObjectClass())){
            CarsIn carsIn = (CarsIn)object;
           for (CarIn carIn : carsIn.getCars()) {
               cz.agents.highway.protobuf.generated.UpdateMessage.Vehicle_update.Builder veh_builder = Vehicle_update.newBuilder();
               veh_builder.setVehicleId(carIn.getId()).setLane(carIn.getLane()).setSpeed(carIn.getV());
               veh_builder.setAcceleration(0).setHighwayX(carIn.getHighwayPoint().x).setHighwayY(carIn.getHighwayPoint().y);
               veh_builder.setPosX(carIn.getPosition().x).setPosY(carIn.getHighwayPoint().y);
               
               //updateTime
               updateTime = (float) Math.min(updateTime,carIn.getUpdateTime());
               updateBuilder.addUpdates(veh_builder.build());
            }
           
        }
      
		return Message.newBuilder().setUpdate(updateBuilder.setUpdateTime(updateTime).build()).build();
	}

    public CarsIn decode(Message pm) throws InvalidProtocolBufferException {
        if (!pm.hasUpdate()) {
            return null;
        }

        Update update = pm.getUpdate();
        CarsIn carUpdates = new CarsIn();
        for (Vehicle_update vu : update.getUpdatesList()) {
            double time = update.getUpdateTime();
            carUpdates.add(new CarIn(vu.getVehicleId(), time, new Point2d(vu.getPosX(), vu
                    .getPosY()), new Point2d(vu.getHighwayX(), vu.getHighwayY()), vu.getSpeed(), vu
                    .getLane()));
        }

        // System.out.println(carUpdates);

        return carUpdates;
    }

    public Class<CarsIn> getObjectClass() {
        return CarsIn.class;
    }

}
