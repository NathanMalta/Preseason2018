package org.usfirst.frc.team4541.profiling;

import org.usfirst.frc.team4541.robot.Robot;

public class RobotState {

	double xPos = 0;
	double yPos = 0;
	double heading = 0;
	double rWheelTravel = 0;
	double lWheelTravel = 0;
	double rotationsPerInch = 0;
	boolean isLocked = false;

	public RobotState(double x, double y, double h, double rWT, double lWT) {
		xPos = x; // in inches
		yPos = y; // in inches
		heading = h; // in radians
		rWheelTravel = rWT; // in rotations
		lWheelTravel = lWT; // in rotations
		// rotationsPerInch = 0.1475;
		rotationsPerInch = 0.08; // in rotations/inch

	}

	public void updatePos(double newRTravel, double newLTravel, double newHeading) {
		double dR = newRTravel - rWheelTravel;
		double dL = newLTravel - lWheelTravel;
		double dH = rotate(getInverseOfAngle(heading), newHeading);
		double epsilon = 0.0000001; // threshold between circular and linear estimations
		if (dH > epsilon) { // estimate change in position using constant radius math
			double radius = (dR + dL) / (2 * dH);
			double mag = 2 * radius * Math.sin(dH / 2);
			double XMag = mag * Math.cos(newHeading);
			double YMag = mag * Math.sin(newHeading);
			xPos += XMag;
			yPos += YMag;
		} else { // estimate change in position using linear distance between points
			double mag = (dR + dL) / 2;
			yPos += Math.sin(newHeading) * mag;
			xPos += Math.cos(newHeading) * mag;
		}
		heading = newHeading;
		rWheelTravel = newRTravel;
		lWheelTravel = newLTravel;

	}

	/*
	 * Effectively undoes the rotation by an angle; If a line segment is rotated by
	 * the angle and then its inverse, it will have the same heading as before being
	 * rotated
	 */
	public double getInverseOfAngle(double angleRad) {
		return Math.atan2((-1 * Math.sin(angleRad)), Math.cos(angleRad));
	}

	/*
	 * Rotates an a1 by a2 using a rotation matrix
	 */
	public double rotate(double a1, double a2) {
		return Math.atan2(Math.cos(a1) * Math.sin(a2) + Math.sin(a1) * Math.cos(a2),
				Math.cos(a1) * Math.cos(a2) - Math.sin(a1) * Math.sin(a2));
	}

	/*
	 * Converts the number of wheel rotations into the distance traveled by the
	 * robot in inches
	 */
	public double rotationToInches(double rot) {
		return rot / rotationsPerInch;
	}

	/*
	 * Get the absolute x position of the robot from dead reckoning
	 */
	public double getXPos() {
		return rotationToInches(xPos); // in inches
	}

	/*
	 * Get the absolute y position of the robot from dead reckoning
	 */
	public double getYPos() {
		return rotationToInches(yPos); // in inches
	}

	/*
	 * Get the net distance traveled by the right wheels of the robot
	 */
	public double getRightWheelTravel() {
		return rWheelTravel; // in inches
	}

	/*
	 * Get the net distance traveled by the left wheels of the robot
	 */
	public double getLeftWheelTravel() {
		return lWheelTravel; // in inches
	}

	/*
	 * Get the heading of the robot (from the gyroscope at the time of the last
	 * update)
	 */
	public double getHeading() {
		return heading; // in radians
	}

	public double getRightWheel() {
		return Robot.drivetrain.getRightVel();
	}

	public double getLeftWheel() {
		return Robot.drivetrain.getLeftVel();
	}

	public double getX() {
		return xPos;
	}

	public double getY() {
		return yPos;
	}

	public void start() {
		Thread t = new Thread(() -> {
			while (true) {

				this.updatePos(getRightWheel(), getLeftWheel(), getHeading());

			}
		});
		t.start();
	}

}
