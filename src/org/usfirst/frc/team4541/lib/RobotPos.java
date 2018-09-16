package org.usfirst.frc.team4541.lib;

import org.usfirst.frc.team4541.robot.Constants;

public class RobotPos {
	Point position;
	double heading; //heading in radians
	double rVel;
	double lVel;
	
	public RobotPos(double robotX, double robotY, double heading, double rVel, double lVel) { //heading in radians
		this(new Point(robotX, robotY), heading, rVel, lVel);
	}
	
	public RobotPos(Point pos, double heading, double rVel, double lVel) { // heading in radians
		this.position = pos;
		this.heading = heading;
		this.rVel = rVel;
		this.lVel = lVel;
	}
	
	public double getRightVel() {
		return this.rVel;
	}
	
	public double getLeftVel() {
		return this.lVel;
	}
	
	public double getDifferential() {
		return this.getRightVel() - this.getLeftVel();
	}
	
	public double getVelocity() {
		return (this.rVel + this.lVel) / 2.0;
	}
	
//	public Point getLookaheadPoint() { // Incorporate velocity later?
//		// get a point which is kLookaheadDistance inches in front of the robot
//		Point lookahead = new Point(Math.cos(heading) * Constants.kLookaheadDistance, Math.sin(heading) * Constants.kLookaheadDistance);
//		// add that point to the robot's current position
//		return Point.addPoints(this.position, lookahead);
//	}
//	
	public Point getVelocityLookaheadPoint(double dt) {
		// get where the robot will be next update
		double dist = dt * this.getVelocity();
		Point lookahead = new Point(Math.cos(heading) * dist, Math.sin(heading) * dist);
		// add that point to the robot's current position
		return Point.addPoints(this.position, lookahead);
	}
	
	@Override
	public String toString() {
		return this.position.getX() + "," + this.position.getY() + "," + this.heading + "," + this.lVel + "," + this.rVel; 
	}
}
