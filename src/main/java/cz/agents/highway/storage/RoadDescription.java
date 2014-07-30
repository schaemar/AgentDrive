package cz.agents.highway.storage;

import cz.agents.highway.environment.roadnet.Network;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;

public class RoadDescription {


    private final Network roadNetwork;
    private Point3d lastPoint = null;
	
//	class PointOrdering implements Comparator<Point3d> {
//		public int compare(Point3d o1, Point3d o2) {
//			int xCmp = (int)Math.signum(o1.getX() - o2.getX());
//			if(xCmp != 0){
//				return xCmp;
//			}
//			int yCmp = (int)Math.signum(o1.getY() - o2.getY());
//			if(yCmp != 0){
//				return yCmp;
//			}
//			return 0;
//		}
//	}
	
//	private final TreeMap<Point3d, Double> profile = new TreeMap<Point3d, Double>(new PointOrdering());
	private final List<Line> lines = new ArrayList<Line>();
	private final List<RoadObject> obstacles = new ArrayList<RoadObject>();


    public RoadDescription(Network roadNetwork) {
        this.roadNetwork = roadNetwork;
    }

    public Network getRoadNetwork() {
        return roadNetwork;
    }
//	public void addPoint(Point3d p) {
//		double dist = 0;
//		if (lastPoint != null) {
//			Line line = new Line(lastPoint, p);
//			lines.add(line);
//			if(line.isTaper()){
//				// add an obstacle to every taper lane
//				for(int i = (int)lastPoint.z; i > p.z; i--){
//					//TODO this is only approx position...
//					Point3f obstPoint = new Point3f((float)p.x, (float)p.y, 0f);
//					obstacles.add(new RoadObject(-1, 0f ,i-1, obstPoint, new Vector3f()));
//				}
//			}
//			dist = profile.get(lastPoint) + distance(p, lastPoint);
//		}
//		profile.put(p, dist);
//		lastPoint = p;
//	}
	
	private double distance(Point3d p1, Point3d p2){
//		double dx = (p1.x - p2.x);
		return (p1.y - p2.y);
//		return Math.sqrt(dx*dx + dy*dy);
	}
	
//	public void addPoints(Collection<Point3d> ps) {
//		for(Point3d p: ps){
//			addPoint(p);
//		}
//	}
	
	public List<Line> getLines() {
		return lines;
	}
	
	public List<RoadObject> getObstacles() {
		return obstacles;
	}
	
	public double distance(Point2d position){
//		Point3d toSearch = new Point3d(position.x, position.y, 0);
//		Entry<Point3d, Double> entry = profile.floorEntry(toSearch);
////		if(entry == null){
////		    entry = profile.ceilingEntry(toSearch);
////		}
		return -position.y;
	}
	
//	public Point3d getNearestHighwayPoint(Point2d position){
//		Point3d toSearch = new Point3d(position.x, position.y, 0);
//		return profile.floorKey(toSearch);
//	}

	public static class Line {
		public final Point3d a;
		public final Point3d b;

		public Line(Point3d a, Point3d b) {
			this.a = a;
			this.b = b;
		}
		
		public boolean isTaper(){
			return a.z > b.z;
		}
		
		public boolean isExtension(){
			return a.z < b.z;
		}
	}

}
