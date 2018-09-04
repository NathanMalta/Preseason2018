package org.usfirst.frc.team4541.robot;

public class Constants { //TODO: play with these values once we get a robot
	public static double kLookaheadDistance = 12.0; //in inches; use a fixed lookahead for now; 254 uses a varying lookahead based on speed
	public static double kPathPursuitTolerance = 1.0; //in inches; get within 1 inch of the endpoint prior to moving on to the next segment
	public static double kMaxVelocity = 60; //in inches/sec; the max velocity the robot can get to when traveling a path
	public static double kMaxAccel = 24; //in inches/sec^2; the max acceleration the robot can be commanded to experience when traveling a path
	public static double kMaxJerk = 24; //in inches/sec^3; the max jerk the robot can be commanded to experience when traveling a path
	public static double defaultDt = 0.02; // in seconds; the default dt for pathfinding calculations
	
	public static final double kNeutralDeadband = 0.01;
	public static final int kTimeoutMs = 10;
	public static final double kSensorUnitsPerRotation = 854.3; //pulses per foot 854.3 math; 876.7 experimentally, 848.18- 4 times 64
	
}