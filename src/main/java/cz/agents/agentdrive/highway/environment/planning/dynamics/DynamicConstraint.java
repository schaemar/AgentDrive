package cz.agents.agentdrive.highway.environment.planning.dynamics;

import cz.agents.agentdrive.highway.environment.planning.euclid4d.Point4d;
import tt.euclid2d.Point;

import java.util.Set;

/**
 *
 * Created by wmatex on 4.4.15.
 */
public interface DynamicConstraint {
    public Set<Point4d> expand(final Point4d current, final Point toExpand);
}
