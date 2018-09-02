package org.usfirst.frc.team4541.profiling;

public class RobotState {

	double xPos = 0;
	double yPos = 0;
	double heading = 0;
	double rWheelTravel = 0;
	double lWheelTravel = 0;
	double rotationsPerInch = 0;

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
	
	public double getInverseOfAngle(double angleRad) {
		return atan2((-1 *Math.sin(angleRad)), Math.cos(angleRad));
	}

}
