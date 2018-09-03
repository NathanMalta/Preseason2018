package org.usfirst.frc.team4541.lib;

public interface Segment {
	
	public Point getClosestPointOnSegment(Point lookaheadPoint);
	
	public Point getStartPoint();
	
	public Point getEndPoint();
	
}
