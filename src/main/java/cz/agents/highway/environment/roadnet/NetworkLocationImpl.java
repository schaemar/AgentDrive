package cz.agents.highway.environment.roadnet;

import cz.agents.highway.environment.roadnet.network.NetworkLocation;
import tt.euclid2d.region.Rectangle;

import javax.vecmath.Point2d;
import javax.vecmath.Point2f;

public class NetworkLocationImpl implements NetworkLocation{


    private Point2f offset;
    private Rectangle convertedBoundary;
    private Rectangle origBoundary;
    private String projection;

    public NetworkLocationImpl(Point2f offset, Rectangle convertedBoundary, Rectangle origBoundary, String projection) {
        this.offset = offset;
        this.convertedBoundary = convertedBoundary;
        this.origBoundary = origBoundary;
        this.projection = projection;
    }

    @Override
    public Point2f getOffset() {
        return offset;
    }

    @Override
    public Rectangle getConvertedBoundary() {
        return convertedBoundary;
    }

    @Override
    public Rectangle getOrigBoundary() {
        return origBoundary;
    }

    @Override
    public String getProjection() {
        return projection;
    }
}
