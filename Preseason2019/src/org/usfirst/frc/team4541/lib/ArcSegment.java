package org.usfirst.frc.team4541.lib;

public class ArcSegment implements Segment {
	
	private Point startPoint;
	private Point endPoint;
	private Point centerPoint;
	
	private Point deltaStart;
	private Point deltaEnd;
	
	public ArcSegment(Point startPoint, Point endPoint, Point centerPoint) {
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.centerPoint = centerPoint;
		
		this.deltaStart = Point.getDelta(centerPoint, startPoint);
		this.deltaEnd = Point.getDelta(centerPoint, endPoint);
	}
	
	/*
	 * Gets the closest point on the arc segment to the given point.  Approach was modified from
	 * Team 254's on github:
	 * https://github.com/Team254/FRC-2017-Public/blob/master/src/com/team254/lib/util/control/PathSegment.java
	 * 
	 * @param lookaheadPoint: the robot's position + lookahead
	 */
	@Override
	public Point getClosestPointOnSegment(Point lookaheadPoint) {
		Point deltaPose = Point.getDelta(centerPoint, lookaheadPoint);
		double scale = deltaStart.getHypot() / deltaPose.getHypot();
		deltaPose = new Point(deltaPose.getX() * scale, deltaPose.getY() * scale);
		
		if (Point.cross(deltaPose, deltaStart) * Point.cross(deltaPose, deltaEnd) < 0) {
			return new Point(centerPoint.getX() + deltaPose.getX(), centerPoint.getY() + deltaPose.getY());
		} else {
			Point startDist = Point.getDelta(deltaPose, startPoint);
			Point endDist = Point.getDelta(deltaPose, endPoint);
			return (endDist.getHypot() < startDist.getHypot()) ? endPoint : startPoint;
		}
	}

	@Override
	public Point getStartPoint() {
		return this.startPoint;
	}

	@Override
	public Point getEndPoint() {
		return this.endPoint;
	}
	
	public Point getCenterPoint() {
		return this.centerPoint;
	}

}
