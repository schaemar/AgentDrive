package cz.agents.highway.protobuf.factory;

import java.util.ArrayList;
import java.util.Collection;
import javax.vecmath.Point3d;

import com.google.protobuf.InvalidProtocolBufferException;

import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.highway.protobuf.generated.InitMessage.Init;
import cz.agents.highway.protobuf.generated.InitMessage.Init_Message;
import cz.agents.highway.protobuf.generated.MessageContainer.Message;
import cz.agents.highway.storage.InitIn;

public class InitMessageFactory implements FactoryInterface<InitIn, Message> {

	public Message encode(Object object) {
		InitIn initin = (InitIn)object;
		cz.agents.highway.protobuf.generated.InitMessage.Init.Builder builder = Init.newBuilder();
		
		for (Point3d point : initin.getPoints()) {
		    builder.addInit(Init_Message.newBuilder().setHighwayX(point.x).setHighwayY(point.y).setNumberOfLanes(point.z).build());
		}
		
		return Message.newBuilder().setInit(builder.build()).build();
	}

	public InitIn decode(Message pm) throws InvalidProtocolBufferException {
		if(!pm.hasInit()){
			return null;
		}
		Init init = pm.getInit();
		
		Collection<Point3d> points = new ArrayList<Point3d>();
		
		for(Init_Message init_Message: init.getInitList()){
			points.add(new Point3d(init_Message.getHighwayX(), init_Message.getHighwayY(), init_Message.getNumberOfLanes()));
		}
		
		InitIn initIn = new InitIn(points);
		
//		System.out.println(initIn);
		
		return initIn;
	}

	public Class<InitIn> getObjectClass() {
		return InitIn.class;
	}

}
