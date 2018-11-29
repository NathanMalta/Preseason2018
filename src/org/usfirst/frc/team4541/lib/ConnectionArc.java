package org.usfirst.frc.team4541.lib;

import org.usfirst.frc.team4541.robot.Constants;

public class ConnectionArc {
	Point startPoint;
	Point endPoint;
	double radius;
	boolean isTurningRight;

	public ConnectionArc(RobotPos rPos, Point endPoint) {
		this.startPoint = rPos.position;
		this.endPoint = endPoint;

		Point robotRelativeGoal = Point.getP1RelativeToP2(this.endPoint, this.startPoint, rPos.heading);

		if (robotRelativeGoal.getX() == 0) {
			this.radius = Double.MAX_VALUE;
		} else {
			this.radius = Math.pow(robotRelativeGoal.getHypot(), 2) / (2 * Math.abs(robotRelativeGoal.getX()));
		}
		if (robotRelativeGoal.getX() < 0) {
			this.isTurningRight = true;
		} else {
			this.isTurningRight = false;
		}
	}

	public ConnectionArc(double radius, boolean isTurningRight) {
		this.radius = radius;
		this.isTurningRight = isTurningRight;
	}

	/**
	 * 
	 * @param currentVel:
	 *            the current center velocity of the robot
	 * @param turnRad:
	 *            the radius of the turn desired
	 * @return a left wheel velocity that will allow the robot to make the turn
	 */
	public double getRightVelocityTarget(double currentVel) {
		double a;
		if (this.isTurningRight) {
			a = (this.radius - (Constants.kWheelBase / 2)) / (this.radius + (Constants.kWheelBase / 2));
		} else {
			a = (this.radius + (Constants.kWheelBase / 2)) / (this.radius - (Constants.kWheelBase / 2));
		}
		return (2 * currentVel) / (a + 1);
	}

	/**
	 * 
	 * @param currentVel:
	 *            the current center velocity of the robot
	 * @param turnRad:
	 *            the radius of the turn desired
	 * @return: a right wheel velocity that will allow the robot to make the
	 *          turn
	 */
	public double getLeftVelocityTarget(double currentVel) {
		double a;
		if (this.isTurningRight) {
			a = (this.radius + (Constants.kWheelBase / 2)) / (this.radius - (Constants.kWheelBase / 2));
		} else {
			a = (this.radius - (Constants.kWheelBase / 2)) / (this.radius + (Constants.kWheelBase / 2));
		}
		return (2 * currentVel) / (a + 1);

	}

	public double getLeftVelocityTargetFromRightVelocity(double rVel) {
		double a;
		if (this.isTurningRight) {
			a = (this.radius - (Constants.kWheelBase / 2)) / (this.radius + (Constants.kWheelBase / 2));
		} else {
			a = (this.radius + (Constants.kWheelBase / 2)) / (this.radius - (Constants.kWheelBase / 2));
		}
		return a * rVel;
	}

	public double getRightVelocityTargetFromLeftVelocity(double lVel) {
		double a;
		if (this.isTurningRight) {
			a = (this.radius + (Constants.kWheelBase / 2)) / (this.radius - (Constants.kWheelBase / 2));
		} else {
			a = (this.radius - (Constants.kWheelBase / 2)) / (this.radius + (Constants.kWheelBase / 2));
		}
		return a * lVel;
	}

}
