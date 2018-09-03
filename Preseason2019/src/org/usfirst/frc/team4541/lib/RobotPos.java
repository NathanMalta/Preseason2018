package org.usfirst.frc.team4541.lib;

import org.usfirst.frc.team4541.robot.Constants;

public class RobotPos {
	Point position;
	double heading; //heading in radians
	
	public RobotPos(double robotX, double robotY, double heading) { //heading in radians
		this(new Point(robotX, robotY), heading);
	}
	
	public RobotPos(Point pos, double heading) { // heading in radians
		this.position = pos;
		this.heading = heading;
	}
	
	public Point getLookaheadPoint() { // Incorporate velocity later?
		// get a point which is kLookaheadDistance inches in front of the robot
		Point lookahead = new Point(Math.cos(heading) * Constants.kLookaheadDistance, Math.sin(heading) * Constants.kLookaheadDistance);
		// add that point to the robot's current position
		return Point.addPoints(this.position, lookahead);
	}
}
