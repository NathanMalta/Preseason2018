package org.usfirst.frc.team4541.robot;

public class Constants { //TODO: play with these values once we get a robot
	public static final double kLookaheadDistance = 3; //in inches; use a fixed lookahead for now; 254 uses a varying lookahead based on speed
	public static final double kPathPursuitTolerance = 2; //in inches; get within 1 inch of the endpoint prior to moving on to the next segment
	public static final int    kSetpointSmoothingFactor = 15; //amount of values in the moving average filter for turning setpoint
	public static final double kMaxVelocity = 60; //in inches/sec; the max velocity the robot can get to when traveling a path
//	public static final double kMaxJerk = 6; //(NOT CURRENTLY USED) in inches/sec^3; the max jerk the robot can be commanded to experience when traveling a path
	public static final double kDefaultDt = 0.05; // in seconds; the default dt for pathfinding calculations
	
//	public static final double kTurnAccelCoefficient = 0.6; // the percent of acceleration dedicated to turning per ft/sec of differential error - creates max turning accel
//	public static final double kTurnVelCoefficient = 0.2; // the percent of velocity that will be used as a differential per radian of angular error - creates vel diff setpoint
	public static final double kTurningVelocity = 60; 
	
	public static final double kMaxAccelSpeedUp = 10; //in inches/sec^2; the max acceleration the robot can be commanded to experience when traveling a path
//	public static final double kMaxAccelTurning = 1000;
	public static final double kMaxTurningVelocity = 20;
	
	public static final double kWheelDiameter = 6; // in inches; the diameter of the drive wheels
	public static final double kWheelBase = 30; //in inches; the distance between the left and right drive wheels
	public static final double kTrackScrubFactor = 0.924;
	public static final double kStopSteeringDistance = 1;
	
	public static final double kNeutralDeadband = 0.01;
	public static final int kTimeoutMs = 10;
	public static final double kSensorUnitsPerFt = 854.3; //pulses per foot 854.3 math; 876.7 experimentally, 848.18- 4 times 64
	
}