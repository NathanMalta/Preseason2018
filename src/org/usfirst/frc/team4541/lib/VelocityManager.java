package org.usfirst.frc.team4541.lib;

import org.usfirst.frc.team4541.robot.Constants;

public class VelocityManager { //creates a trapezoidal velocity curve for the robot to follow
	double dt;
	double lastUpdateTime;
	SynchronousPIDF pid;
	MovingAverage avg;
	
	boolean isHeadingFrozen;
	double frozenHeading = 0;
	public VelocityManager() {
		this.dt = -1;
		this.lastUpdateTime = -1;
//		this.pid = new SynchronousPIDF(13, 0, 6); //at 50 hz (0.02 ms)
		this.pid = new SynchronousPIDF(150, 0, 0); //at 20 hz (0.05 ms)
		this.pid.setContinuous(true);
		this.pid.setInputRange(-Math.PI, Math.PI);
		this.pid.setOutputRange(-Constants.kMaxTurningVelocity, Constants.kMaxTurningVelocity);
//		this.avg = new MovingAverage(Constants.kSetpointSmoothingFactor);
		this.avg = new MovingAverage(10);
		this.isHeadingFrozen = false;
	}
	
	/*
	 * returns a RobotCmd containing the desired left and right velocities for the wheels
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public RobotCmd getVelCmd(Segment segment, RobotPos state) {
		if (dt == -1) { 
			//on the first update, setup the dt calculations
			lastUpdateTime = System.currentTimeMillis();
			this.dt = Constants.kDefaultDt;
		} else { 
			// update dt based on time elapsed since last update
//			this.updateDt(); //TODO: remove comment - just to allow speedup in simulation
		}
		double overallVel = this.getOverallVelocityTarget(segment, state);
//		double differential = this.getNextDifferential(Constants.kMaxAccelTurning, state.getDifferential(), 
//				this.getVelocityDifferentialSetpoint(segment, state)) / 2;
		
		double differential = this.getVelocityDifferentialSetpoint(segment, state) / 2;
		
		
//		Point targetPoint = segment.getLookaheadPoint(state.position, Constants.kLookaheadDistance);
//		double newHeading = state.heading + this.getAngleChangeAfterDiffDecrease(state.getDifferential());
//		double angleError = MathHelper.angleToNegPiToPi(Point.getAngleNeeded(state.position, targetPoint) - newHeading);
		
		//Just for testing, remove later
//		Point lookahead = segment.getLookaheadPoint(state.position, Constants.kLookaheadDistance);
//		double angleError = state.heading - Point.getAngleNeeded(state.position, lookahead);
	
//		if (differential > 0 && angleError < 0 || differential < 0 && angleError > 0) {
//			System.out.println(state.position.getX() + "," +  state.position.getY() + ",," + lookahead.getX() + "," + lookahead.getY());
//		} else {
//			System.out.println(",," + state.position.getX() + "," + state.position.getY() + "," + lookahead.getX() + "," + lookahead.getY());
//		}
		
		return new RobotCmd(overallVel - differential, overallVel + differential); //: make sure robot turns in the right direction
	}
	
	/*
	 * returns the heading the robot would be at if the current differential was slowed down to 0
	 * 
	 * @param currentDifferential: the current differential of the robot
	 */
//	public double getAngleChangeAfterDiffDecrease(double currentDifferential) {
//		double angularVel = -1 * currentDifferential / Constants.kWheelBase;
//		double secondsOfDecrease = 1 + Math.abs(currentDifferential) / Constants.kMaxAccelTurning;
//		
//		double angleChange = (angularVel * secondsOfDecrease) / 2;
//		
//		return angleChange;
//	}
	
	
	/*
	 * returns the velocity desired for the robot as a whole
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public double getOverallVelocityTarget(Segment segment, RobotPos state) {
		
		if (this.isHeadingFrozen) {
			this.getNextVelocity(Constants.kMaxAccelSpeedUp, state.getVelocity(), 0);
		}
		
		if (segment.isAcceleratingToEndpoint()) {
			double requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), segment.getEndVelocity(), segment.getDistanceToEndpoint(state.getVelocityLookaheadPoint(dt)));
			return this.getNextVelocity(requiredAccelToFinish, state.getVelocity(), segment.getEndVelocity());
		}
		if (this.shouldAccelToEndPoint(segment, state)) {
			//The robot must accelerate/decelerate to reach the velocity desired by the segment's endpoint
			segment.setIsAcceleratingToEndpoint(true);
			return this.getNextVelocity(Constants.kMaxAccelSpeedUp, state.getVelocity(), segment.getEndVelocity());
		} else {
			//The robot must accelerate/decelerate to reach the max speed of the path
			return this.getNextVelocity(Constants.kMaxAccelSpeedUp, state.getVelocity(), segment.getMaxVelocity());
		}
	}
	
	/*
	 * returns if the robot should accelerate/decelerate to get to the velocity desired at the endpoint
	 * 
	 * @param segment: the segment the robot is currently following
	 * @param state: the current position + velocity of the robot
	 */
	public boolean shouldAccelToEndPoint(Segment segment, RobotPos state) {
//		double remainingDist = this.getDistanceRemaining(segment, state);
//		double distanceToAccel = this.getDistanceNeededToAccel(this.getAccelForSpeedUp(segment, state), state.getVelocity(), segment.getEndVelocity());
//		return remainingDist <= distanceToAccel;
		double requiredAccelToFinish = this.getAccelNeededToGetToVelByPoint(state.getVelocity(), segment.getEndVelocity(), segment.getDistanceToEndpoint(segment.getLookaheadPoint(state.position, Math.abs(state.getVelocity() * this.dt))));
		return requiredAccelToFinish >= Constants.kMaxAccelSpeedUp;
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
	 * returns the acceleration required to achieve a certain velocity from the current velocity by a given
	 * distance
	 * 
	 * @param v2: current velocity
	 * @param v2: desired velocity
	 * @param dist: distance desired
	 */
	public double getAccelNeededToGetToVelByPoint(double v1, double v2, double dist) {
		double deltaV = Math.abs(v2 - v1);
		
		double secondsRequired = (2 * dist) / deltaV;
		
		return deltaV / secondsRequired;
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
		if (Math.abs(delta) <= (this.dt * maxSpeedAccel)) {
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
		this.lastUpdateTime = System.currentTimeMillis();
	}
	
	/*
	 * Get the desired differential in velocity for the robot's wheels
	 * 
	 * @param segment: the current segment the robot is following
	 * @param state: the current position + velocity of the robot
	 */
	public double getVelocityDifferentialSetpoint(Segment segment, RobotPos state) {
		Point targetPoint = segment.getLookaheadPoint(state.position, Constants.kLookaheadDistance);

//		double newHeading = state.heading + this.getAngleChangeAfterDiffDecrease(state.getDifferential());
		
//		double angleError = MathHelper.angleToNegPiToPi(Point.getAngleNeeded(state.position, targetPoint) - state.heading);
//		double angleError = MathHelper.angleToNegPiToPi(newHeading - Math.PI / 4);
		
//		System.out.println( MathHelper.angleToNegPiToPi(newHeading) + "," + MathHelper.angleToNegPiToPi(state.heading) + "," + angleError);
//		System.out.println(MathHelper.angleToNegPiToPi(state.heading) + "," + Point.getAngleNeeded(state.position, targetPoint));
//		System.out.println(angleError);
//		System.out.println(state.position.getX() + "," + state.position.getY() + "," + targetPoint.getX() + "," + targetPoint.getY());
		double setpoint;
		if (!this.isHeadingFrozen) {
			setpoint = Point.getAngleNeeded(state.position, targetPoint);
		} else {
			setpoint = this.frozenHeading;
		}
		
		avg.add(setpoint);
		this.pid.setSetpoint(avg.getAvg());
		this.pid.calculate(state.heading, this.dt);
//		System.out.println(state.heading + "," + pid.getSetpoint() + "," + pid.getError());
		double output = this.pid.get();
		
		return output;
		
		
//		return Constants.kTurningVelocity * Math.max(-1, Math.min(1, angleError * Constants.kTurnVelCoefficient));

		
		
//		if (angleAfterDecrease < angleError) { //TODO: make work with different signs
//			
//			if (angleError > 0) {
//				return Constants.kMaxVelocity; 
//			} else if (angleError < 0){
//				return -Constants.kMaxVelocity;
//			} else {
//				return 0;
//			}
//			
//		} else { 
//			//we will overshoot so decrease absolute value of differential
//			if (state.getDifferential() > 0) {
//				return state.getDifferential() + Constants.kMaxAccelTurning * this.dt;
//			} else if (state.getDifferential() < 0) {
//				return state.getDifferential() - Constants.kMaxAccelTurning * this.dt;
//			} else {
//				return 0;
//			}
//		}
		
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
		double delta = currentDiff - desiredDiff;
		if (Math.abs(delta) <= (this.dt * maxTurnAccel)) {
			// if moving to the desired velocity is allowed within acceleration constraints, 
			// return the desired velocity
			return desiredDiff;
			
		} else if (delta > 0) {
			// The next velocity is greater than the current velocity, speed up the robot
			return currentDiff - (this.dt * maxTurnAccel);
		} else if (delta < 0) {
			// The next velocity is less than the current velocity, slow down the robot
			return currentDiff + (this.dt * maxTurnAccel);
		} else {
			// The next velocity is the same as the current velocity, maintain current velocity
			return currentDiff;
		}
	}
	
	public void freezeHeading(double heading) {
		this.isHeadingFrozen = true;
		this.frozenHeading = heading;
	}
	
}
