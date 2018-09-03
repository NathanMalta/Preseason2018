package org.usfirst.frc.team4541.lib;

public class LineSegment implements Segment {
	
	private Point startPoint;
	private Point endPoint;
	
	public LineSegment(Point startPoint, Point endPoint) {
		this.startPoint = startPoint;
		this.endPoint = endPoint;
	}
	
	/*
	 * Gets the closest point on the line segment to the given point.  Approach was modified from
	 * Joshua's on stack overflow:
	 * https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
	 * 
	 * @param lookaheadPoint: the robot's position + lookahead
	 */
	@Override
	public Point getClosestPointOnSegment(Point lookaheadPoint) {
		double A = lookaheadPoint.getX() - this.startPoint.getX();
		double B = lookaheadPoint.getY() - this.startPoint.getY();
		double C = this.endPoint.getX() - this.startPoint.getX();
		double D = this.endPoint.getY() - this.startPoint.getY();
		
		double dot = A * C + B * D;
		double len_sq = C * C + D * D;
		
		double param = -1;
		if (len_sq != 0) {
			param = dot / len_sq;
		}
		
		double xx, yy;
		
		if (param < 0) {
			xx = this.startPoint.getX();
			yy = this.startPoint.getY();
		}
		else if (param > 1) {
            xx = this.endPoint.getX();
	        yy = this.endPoint.getY();
	    }
	    else {
	        xx = this.startPoint.getX() + param * C;
	        yy = this.startPoint.getY() + param * D;
	    }
		
		return new Point(xx, yy);
	}

	@Override
	public Point getStartPoint() {
		return this.startPoint;
	}

	@Override
	public Point getEndPoint() {
		return this.endPoint;
	}
	
}
