package org.usfirst.frc.team4541.lib;

public interface Segment {
	
	public Point getClosestPointOnSegment(Point lookaheadPoint);
	
	public Point getStartPoint();
	
	public Point getEndPoint();
	
	public double getDistanceToEndpoint(Point lookaheadPos);
	
	public double getMaxVelocity();
	
	public double getEndVelocity();
	
	public void setIsAcceleratingToEndpoint(boolean isAccel);
	
	public boolean isAcceleratingToEndpoint();
	
}
