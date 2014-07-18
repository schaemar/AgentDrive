package cz.agents.highway.agent;

import rvolib.Vector2;

import javax.vecmath.Point3f;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3f;

/**
 * Created by martin on 15.7.14.
 */
public class ORCAUtil {
    public static Vector2 vector3fToVector2(Tuple3f position) {
        return new Vector2(position.x, position.y);
    }

    public static Vector3f vector2ToVector3f(Vector2 vector){
        return new Vector3f(vector.x(),vector.y(),0);
    }

}
