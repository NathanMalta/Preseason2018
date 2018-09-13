package org.usfirst.frc.team4541.lib;

import java.util.ArrayList;

public class ConnectionArc {
	Point startPoint;
	Point endPoint;
	Point centerPoint;
	RobotPos rPos;
	double radius;
	double dt;
	public ConnectionArc(RobotPos rPos, Point endPoint, double dt) {
		this.startPoint = rPos.position;
		this.endPoint = endPoint;
		this.rPos = rPos;
		this.dt = dt;
		
		Point delta = Point.getDelta(startPoint, endPoint);
		this.radius = delta.getHypot() / 2;
		
		//create all posible arcs joining the start and end points
		ArrayList<Point> possibleCenters = new ArrayList<Point>();
		possibleCenters.add(new Point(startPoint.getX(), startPoint.getY() + this.radius));
		possibleCenters.add(new Point(startPoint.getX(), startPoint.getY() - this.radius));
		possibleCenters.add(new Point(startPoint.getX() + this.radius, startPoint.getY()));
		possibleCenters.add(new Point(startPoint.getX() - this.radius, startPoint.getY()));
		
		ArrayList<Point> centers = new ArrayList<Point>();
		//remove invalid center candidates
		for (Point center : possibleCenters) {
			if (MathHelper.areApproxEqual(Point.getDistance(center, startPoint), Point.getDistance(center, endPoint))) {
				centers.add(center);
			}
			System.out.println(Point.getDistance(center, startPoint) + " " + Point.getDistance(center, endPoint));
		}
		
		//get lookaheads for all centers
		ArrayList<Point> lookaheads = new ArrayList<Point>();
		for (Point center : centers) {
			lookaheads.add(getLookaheadPoint(rPos, endPoint, center, this.radius, dt));
		}
		
		//get the index of the lookahead that will take the least turning to achieve
		double theta = Point.getAngleNeeded(rPos.position, lookaheads.get(0));
		int bestIndex = 0;
		for (Point lookahead : lookaheads) {
			double currentTheta = Point.getAngleNeeded(rPos.position, lookaheads.get(0));
			if (theta > currentTheta) {
				theta = currentTheta;
				bestIndex = lookaheads.indexOf(lookahead);
			}
		}
		
		//set the best centerpoint to the center of the arc
		this.centerPoint = possibleCenters.get(bestIndex);
	}
	
	public Point getLookaheadPoint() {
		return getLookaheadPoint(this.rPos, this.endPoint, this.centerPoint, this.radius, this.dt);
	}
	
	public static Point getLookaheadPoint(RobotPos rPos, Point endPoint, Point centerPoint, double radius, double dt) {
		double lookahead = rPos.getVelocity() * dt;
		double theta = lookahead / radius;
		
		// get the absolute distance between the lookahead and the robot
		double oppLen = Point.getOppSideLength(radius, radius, theta);
		// then create distance from start point with trig
		// the angle of the triangle adjacent to the isosceles triangle connecting the center, start point, and lookahead
		double theta2 = (360 - theta) / 2;
		Point delta = new Point(Math.sin(theta2) * oppLen, Math.cos(theta2) * oppLen);
		
		//check all 4 possible triangles with the same delta away from the start point
		//to find which triangle has a point that lies on the arc
		Point testPoint = new Point(rPos.position.getX() + delta.getX(), rPos.position.getY() + delta.getY());
		if (doesPointLieOnArc(rPos.position, endPoint, centerPoint, testPoint)) {
			return testPoint;
		}
		
		testPoint = new Point(rPos.position.getX() + delta.getX(), rPos.position.getY() - delta.getY());
		if (doesPointLieOnArc(rPos.position, endPoint, centerPoint, testPoint)) {
			return testPoint;
		}
		
		testPoint = new Point(rPos.position.getX() - delta.getX(), rPos.position.getY() + delta.getY());
		if (doesPointLieOnArc(rPos.position, endPoint, centerPoint, testPoint)) {
			return testPoint;
		}
		
		testPoint = new Point(rPos.position.getX() - delta.getX(), rPos.position.getY() - delta.getY());
		if (doesPointLieOnArc(rPos.position, endPoint, centerPoint, testPoint)) {
			return testPoint;
		}
		
		return null;
	}
	
	public static boolean doesPointLieOnArc(Point startPoint, Point endPoint, Point centerPoint, Point point) {
		double startAngle = Point.getAngleForArc(startPoint, point, centerPoint);
		double endAngle = Point.getAngleForArc(point, endPoint, centerPoint);
		
		double totalAngle = Point.getAngleForArc(startPoint, endPoint, centerPoint);
		// if the arcs between the endpoints equal the total length of the arc, we are on the arc
		return MathHelper.areApproxEqual(startAngle + endAngle, totalAngle);
	}
	
}
