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
	
	public double getVelocityTarget(Segment segment, RobotPos state) {
		if (dt == -1) { 
			//on the first update, setup the dt calculations
			lastUpdateTime = System.currentTimeMillis();
			this.dt = Constants.defaultDt;
		} else { 
			// update dt based on time elapsed since last update
			this.updateDt();
		}
		
		if (this.shouldAccelToEndPoint(segment, state)) {
			//The robot must accelerate/decelerate to reach the velocity desired by the segment's endpoint
			return this.getNextVelocity(state.getVelocity(), segment.getEndVelocity());
		} else if (this.shouldAccelToMaxVel(segment, state)) {
			//The robot must accelerate to reach the max speed of the path
			return this.getNextVelocity(state.getVelocity(), segment.getMaxVelocity());
		} else {
			//The robot is currently at the max speed of the path and doesn't need to change
			return segment.getMaxVelocity();
		}
	}
	
	/*
	 * returns if the robot should accelerate/decelerate to get to the velocity desired at the endpoint
	 * 
	 * @param segment: the segment the robot is currently following
	 * @param state: the current position + velocity of the robot
	 */
	public boolean shouldAccelToEndPoint(Segment segment, RobotPos state) {
		return this.getDistanceRemaining(segment, state) <= this.getDistanceNeededToAccel(state.getVelocity(), segment.getEndVelocity());
	}
	
	/*
	 * returns if the robot should accelerate/decelerate to get to the max velocity of the segment
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public boolean shouldAccelToMaxVel(Segment segment, RobotPos state) {
		return state.getVelocity() < segment.getMaxVelocity();
	}
	
	/*
	 * returns the time (in seconds) required to accelerate/decelerate from velocity 1 to velocity 2
	 *
	 * @param v1: current velocity
	 * @param v2: desired velocity
	 */
	public double getTimeNeededToAccel(double v1, double v2) {
		double delta = Math.abs(v1 - v2);
		return delta / maxAccel;
	}
	
	/*
	 * returns the distance (in inches) required to accelerate/decelerate from velocity 1 to velocity 2
	 *
	 * @param v1: current velocity
	 * @param v2: desired velocity
	 */
	public double getDistanceNeededToAccel(double v1, double v2) {
		double delta = Math.abs(v1 - v2);
		return this.getTimeNeededToAccel(v1, v2) * delta;
	}
	
	/*
	 * Returns the distance required to get to the segment
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public double getDistanceRemaining(Segment segment, RobotPos state) {
		return segment.getDistanceToEndpoint(state.getLookaheadPoint());
	}
	
	/*
	 * Returns the closest velocity the robot can get to the desired velocity
	 * without violating maximum acceleration constraints
	 * 
	 * @param currentVel: the current velocity of the robot
	 * @param desired vel: the desired velocity of the robot
	 */
	public double getNextVelocity(double currentVel, double desiredVel) {
		double delta = desiredVel - currentVel;
		if (Math.abs(delta) <= this.maxAccel) {
			// if the moving to the desired velocity is allowed within acceleration constraints, 
			// return the desired velocity
			return desiredVel;
			
		} else if (delta > 0) {
			// The next velocity is greater than the current velocity, speed up the robot
			return currentVel + (this.dt * this.maxAccel);
		} else if (delta < 0) {
			// The next velocity is less than the current velocity, slow down the robot
			return currentVel - (this.dt * this.maxAccel);
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
}
