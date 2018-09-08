package org.usfirst.frc.team4541.lib;

import org.usfirst.frc.team4541.robot.Constants;

public class VelocityManager { //creates a trapezoidal velocity curve for the robot to follow
	double dt;
	double lastUpdateTime;
	
	double maxAccel;
	double maxJerk;

	public VelocityManager(double maxAccel, double maxJerk) {
		this.dt = -1;
		this.lastUpdateTime = -1;
		
		this.maxAccel = maxAccel;
		this.maxJerk = maxJerk;
	}
	
	/*
	 * returns a RobotCmd containing the desired left and right velocities for the wheels
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public RobotCmd getVelCmd(Segment segment, RobotPos state) {
		double overallVel = this.getOverallVelocityTarget(segment, state);
		double differential = this.getNextDifferential(this.getAccelForTurning(segment, state), state.getDifferential(), 
				this.getVelocityDifferentialSetpoint(segment, state));
		return new RobotCmd(overallVel - differential, overallVel + differential); //TODO: make sure robot turns in the right direction
	}
	
	
	/*
	 * returns the velocity desired for the robot as a whole
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public double getOverallVelocityTarget(Segment segment, RobotPos state) {
		if (dt == -1) { 
			//on the first update, setup the dt calculations
			lastUpdateTime = System.currentTimeMillis();
			this.dt = Constants.kDefaultDt;
		} else { 
			// update dt based on time elapsed since last update
			this.updateDt();
		}
		double maxSpeedAccel = this.getAccelForSpeedUp(segment, state);
		if (this.shouldAccelToEndPoint(segment, state)) {
			//The robot must accelerate/decelerate to reach the velocity desired by the segment's endpoint
			return this.getNextVelocity(maxSpeedAccel, state.getVelocity(), segment.getEndVelocity());
		} else {
			//The robot must accelerate/decelerate to reach the max speed of the path
			return this.getNextVelocity(maxSpeedAccel, state.getVelocity(), segment.getMaxVelocity());
		}
	}
	
	/*
	 * returns if the robot should accelerate/decelerate to get to the velocity desired at the endpoint
	 * 
	 * @param segment: the segment the robot is currently following
	 * @param state: the current position + velocity of the robot
	 */
	public boolean shouldAccelToEndPoint(Segment segment, RobotPos state) {
		return this.getDistanceRemaining(segment, state) <= this.getDistanceNeededToAccel(this.getAccelForSpeedUp(segment, state), state.getVelocity(), segment.getEndVelocity());
	}
	
	/*
	 * returns the time (in seconds) required to accelerate/decelerate from velocity 1 to velocity 2
	 *
	 * @param v1: current velocity
	 * @param v2: desired velocity
	 * @param maxSpeedAccel: the max acceleration allocated to changing robot speed
	 */
	public double getTimeNeededToAccel(double maxSpeedAccel, double v1, double v2) {
		double delta = Math.abs(v1 - v2);
		return delta / maxSpeedAccel;
	}
	
	/*
	 * returns the distance (in inches) required to accelerate/decelerate from velocity 1 to velocity 2
	 *
	 * @param v1: current velocity
	 * @param v2: desired velocity
	 * @param maxSpeedAccel: the max acceleration allocated to changing robot speed
	 */
	public double getDistanceNeededToAccel(double maxSpeedAccel, double v1, double v2) {
		double delta = Math.abs(v1 - v2);
		return this.getTimeNeededToAccel(maxSpeedAccel, v1, v2) * delta;
	}
	
	/*
	 * Returns the distance required to get to the segment
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public double getDistanceRemaining(Segment segment, RobotPos state) {
		return segment.getDistanceToEndpoint(segment.getClosestPointOnSegment(state.getVelocityLookaheadPoint(dt)));
	}
	
	/*
	 * Returns the closest velocity the robot can get to the desired velocity
	 * without violating maximum acceleration constraints
	 * 
	 * @param currentVel: the current velocity of the robot
	 * @param desiredVel: the desired velocity of the robot
	 * @param maxSpeedAccel: the max acceleration allocated to changing robot speed
	 */
	public double getNextVelocity(double maxSpeedAccel, double currentVel, double desiredVel) {
		double delta = desiredVel - currentVel;
		if (Math.abs(delta) <= maxSpeedAccel) {
			// if moving to the desired velocity is allowed within acceleration constraints, 
			// return the desired velocity
			return desiredVel;
			
		} else if (delta > 0) {
			// The next velocity is greater than the current velocity, speed up the robot
			return currentVel + (this.dt * maxSpeedAccel);
		} else if (delta < 0) {
			// The next velocity is less than the current velocity, slow down the robot
			return currentVel - (this.dt * maxSpeedAccel);
		} else {
			// The next velocity is the same as the current velocity, maintain current velocity
			return currentVel;
		}
	}
	
	/*
	 * Update self.dt with the current change in time between updates
	 */
	public void updateDt() {
		this.dt = (System.currentTimeMillis() - lastUpdateTime) / 1000;
		this.lastUpdateTime = System.currentTimeMillis();
	}
	
	/*
	 * Get the max acceleration allocated to correcting the robot's heading
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public double getAccelForTurning(Segment segment, RobotPos state) {
		double diffError = this.getVelocityDifferentialSetpoint(segment, state) - state.getDifferential();
		return this.maxAccel * Math.min(1, Math.abs(diffError * Constants.kTurnAccelCoefficient));
	}
	
	/*
	 * Get the max acceleration allocated to changing the robot's velocity
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public double getAccelForSpeedUp(Segment segment, RobotPos state) {
		double diffError = this.getVelocityDifferentialSetpoint(segment, state) - state.getDifferential();
		return this.maxAccel * (1 - Math.min(1, Math.abs(diffError * Constants.kTurnAccelCoefficient)));
	}
	
	/*
	 * Get the desired differential in velocity for the robot's wheels
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public double getVelocityDifferentialSetpoint(Segment segment, RobotPos state) {
		Point lookahead = segment.getClosestPointOnSegment(state.getLookaheadPoint());
		double angleError = Point.getAngleNeeded(state.position, lookahead) - state.heading;
		return state.getVelocity() * Math.min(1, Math.abs(angleError * Constants.kTurnVelCoefficient));
	}
	
	/*
	 * Returns the closest differential the robot can get to the desired differential
	 * without violating maximum acceleration constraints
	 * 
	 * @param currentDiff: the current velocity differential of the robot
	 * @param desiredDiff: the desired velocity differential of the robot
	 * @param maxTurnAccel: the max acceleration allocated to changing robot heading
	 */
	public double getNextDifferential(double maxTurnAccel, double currentDiff, double desiredDiff) {
		double delta = desiredDiff - currentDiff;
		if (Math.abs(delta) <= maxTurnAccel) {
			// if moving to the desired velocity is allowed within acceleration constraints, 
			// return the desired velocity
			return desiredDiff;
			
		} else if (delta > 0) {
			// The next velocity is greater than the current velocity, speed up the robot
			return desiredDiff + (this.dt * maxTurnAccel);
		} else if (delta < 0) {
			// The next velocity is less than the current velocity, slow down the robot
			return desiredDiff - (this.dt * maxTurnAccel);
		} else {
			// The next velocity is the same as the current velocity, maintain current velocity
			return desiredDiff;
		}
	}
	
}
