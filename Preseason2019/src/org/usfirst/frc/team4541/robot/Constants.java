package org.usfirst.frc.team4541.robot;

public class Constants { //TODO: play with these values once we get a robot
	public static final double kLookaheadDistance = 5.0; //in inches; use a fixed lookahead for now; 254 uses a varying lookahead based on speed
	public static final double kPathPursuitTolerance = 1.0; //in inches; get within 1 inch of the endpoint prior to moving on to the next segment
	public static final double kMaxVelocity = 12; //in inches/sec; the max velocity the robot can get to when traveling a path
	public static final double kMaxAccel = 6; //in inches/sec^2; the max acceleration the robot can be commanded to experience when traveling a path
	public static final double kMaxJerk = 6; //(NOT CURRENTLY USED) in inches/sec^3; the max jerk the robot can be commanded to experience when traveling a path
	public static final double kDefaultDt = 0.02; // in seconds; the default dt for pathfinding calculations
	
	public static final double kTurnAccelCoefficient = 0.5; // the percent of acceleration dedicated to turning per ft/sec of differential error - creates max turning accel
	public static final double kTurnVelCoefficient = 0.1; // the percent of velocity that will be used as a differential per radian of angular error - creates vel diff setpoint
	
	public static final double kWheelDiameter = 10; // in inches; the diamter of the drive wheels
	public static final double kWheelBase = 30; //in inches; the distance between the left and right drive wheels
			
	public static final double kNeutralDeadband = 0.01;
	public static final int kTimeoutMs = 10;
	public static final double kSensorUnitsPerRotation = 854.3; //pulses per foot 854.3 math; 876.7 experimentally, 848.18- 4 times 64
	
}