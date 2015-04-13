package cz.agents.highway.environment.planning.dynamics;

import cz.agents.highway.environment.planning.euclid3d.SpeedPoint;
import cz.agents.highway.environment.planning.euclid4d.Point4d;
import tt.euclid2d.Point;

import java.util.List;

/**
 *
 * Created by wmatex on 4.4.15.
 */
public interface DynamicConstraint {
    public List<Point4d> expand(final Point4d current, final Point toExpand);
}
