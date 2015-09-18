package cz.agents.highway.environment.roadnet.network;

import tt.euclid2d.region.Rectangle;

import javax.vecmath.Point2d;

public interface NetworkLocation {
    Point2d getOffset();

    Rectangle getConvBoundary();

    Rectangle getOrigBoundary();

    String getProjection();
}
