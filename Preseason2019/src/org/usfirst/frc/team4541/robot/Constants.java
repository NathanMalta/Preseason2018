package org.usfirst.frc.team4541.robot;

public class Constants {
	public static double kLookaheadDistance = 12.0; //in inches; use a fixed lookahead for now; 254 uses a varying lookahead based on speed
	public static double kPathPursuitTolerance = 1.0; //in inches; get within 1 inch of the endpoint prior to moving on to the next segment
	
	public static final double kNeutralDeadband = 0.01;
	public static final int kTimeoutMs = 10;
	
	public static final double kSensorUnitsPerRotation = 854.3; //pulses per foot 854.3 math; 876.7 experimentally, 848.18- 4 times 64
	
	
}