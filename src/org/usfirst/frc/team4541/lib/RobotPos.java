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
	
	public double getVelocity() {
		return (this.rVel + this.lVel) / 2.0;
	}
	
	/**
	 * Gives an approximation of the robot's position at the next update
	 * 
	 * @param dt - change in time between updates
	 * @return - expected position
	 */
	public Point getVelocityLookaheadPoint(double dt) {
		// get where the robot will be next update
		double dist = dt * this.getVelocity();
		Point lookahead = new Point(Math.cos(heading) * dist, Math.sin(heading) * dist);
		// add that point to the robot's current position
		return Point.addPoints(this.position, lookahead);
	}
	
//	/**
//	 * 
//	 * 
//	 * @param timeSec - seconds into the future position should be predicted for
//	 * @return - the expected position after timeSec seconds
//	 */
//	public Point getExpectedPosition(double timeSec) {
//		return null;
//	}
	
	@Override
	public String toString() {
		return this.position.getX() + "," + this.position.getY() + "," + this.heading + "," + this.lVel + "," + this.rVel; 
	}
}
