package org.usfirst.frc.team4541.lib;

import java.util.ArrayList;
import org.usfirst.frc.team4541.robot.Constants;

public class Path {
	ArrayList<Segment> segmentList = new ArrayList<Segment>();
	boolean didFinish;
	
	public Path() {
		didFinish = false;
	}
	
	public void addSegment(Segment segment) {
		segmentList.add(segment);
	}
	
	public RobotCmd update(RobotPos robotPos) {
		Segment currentSegment = this.getCurrentSegment();
		Point currentPos = robotPos.position;
		Point lookahead = robotPos.getLookaheadPoint();
		
		if (Point.getDistance(currentSegment.getEndPoint(), currentPos) < Constants.kPathPursuitTolerance) {
			// if the current robot position is within tol of the end point, move on to the next segment
			// or say that the robot is done following the path
			if (this.canMoveOnToNextSegment()) {
				this.moveOnToNextSegment();
			} else {
				this.didFinish = true;
			}
		}
		
		double headingRequired = Point.getAngleNeeded(currentPos, lookahead);
		double velocityRequired = 0; //TODO: implement
		
		return new RobotCmd(headingRequired, velocityRequired);
	}
	
	public boolean canMoveOnToNextSegment() {
		return segmentList.size() > 1;
	}
	
	public void moveOnToNextSegment() {
		segmentList.remove(0);
	}
	
	public Segment getCurrentSegment() {
		return segmentList.get(0);
	}
	
	public boolean isFinished() {
		return didFinish;
	}
	
}
