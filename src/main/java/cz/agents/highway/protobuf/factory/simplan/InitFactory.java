package cz.agents.highway.protobuf.factory.simplan;

import java.util.ArrayList;
import java.util.Collection;

import javax.vecmath.Point3d;

import org.apache.log4j.Logger;

import com.google.protobuf.InvalidProtocolBufferException;

import cz.agents.alite.protobuf.factory.FactoryInterface;
import cz.agents.highway.protobuf.generated.simplan.InitMessage.Init;
import cz.agents.highway.protobuf.generated.simplan.InitMessage.Init.Builder;
import cz.agents.highway.protobuf.generated.simplan.InitMessage.Init_Message;
import cz.agents.highway.protobuf.generated.simplan.MessageContainer.Message;
import cz.agents.highway.protobuf.generated.simplan.VectorProto.Vector;
import cz.agents.highway.storage.InitIn;

public class InitFactory implements FactoryInterface<InitIn, Message> {

    private static final Logger logger = Logger.getLogger(InitFactory.class);

    public Message encode(Object object) {
        if (!object.getClass().equals(InitIn.class)) {
            logger.warn("invalid input object type (" + object.getClass() + ") to be encoded by "
                    + InitFactory.class + " factory");
            return null;
        }
        InitIn initin = (InitIn) object;
        Builder builder = Init.newBuilder();

        for (Point3d point : initin.getPoints()) {
            // z = 0, z is used to store number of lanes
            Vector position = Vector.newBuilder().setX(point.x).setY(point.y).setZ(0.0).build();

            builder.addInit(Init_Message.newBuilder().setHighwayPoint(position)
                    .setNumberOfLanes(point.z).build());
            System.out.println("initfactory");
        }

        return Message.newBuilder().setInit(builder.build()).build();
    }

    public InitIn decode(Message pm) throws InvalidProtocolBufferException {
        if (!pm.hasInit()) {

            logger.debug("invalid input message content to be decoded by "
                    + InitFactory.class + " factory");
            return null;
        }
        Init init = pm.getInit();

        Collection<Point3d> points = new ArrayList<Point3d>();

        for (Init_Message init_Message : init.getInitList()) {
            points.add(new Point3d(init_Message.getHighwayPoint().getX(), init_Message
                    .getHighwayPoint().getY(), init_Message.getNumberOfLanes()));
        }

        InitIn initIn = new InitIn(points);

        return initIn;
    }

    public Class<InitIn> getObjectClass() {
        return InitIn.class;
    }

}
