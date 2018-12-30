/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// CAN addresses
////	//Drive Motors for ankle bot
//	public static final int leftDriveMotor1  = 3; //1
//	public static final int leftDriveMotor2  = 0; //2
//	
//	public static final int rightDriveMotor1 = 1; //0
//	public static final int rightDriveMotor2 = 2; //3
//	//Drive Motors
//    //Intake Motors
//	public static final int intakeMotor1 = 4;
//	public static final int intakeMotor2 = 5;
//    //Elevator Motors
//	public static final int elevatorMotor1 = 6;
//	public static final int elevatorMotor2 = 7;
//	//PDP
//	public static final int PDP = 8;
	
//	//Drive Motors for test bot
	public static final int leftDriveMotor1  = 2;
	public static final int leftDriveMotor2  = 3;
	
	public static final int rightDriveMotor1 = 1;
	public static final int rightDriveMotor2 = 5;
	//Drive Motors
    //Intake Motors
	public static final int intakeMotor1 = 7;
	public static final int intakeMotor2 = 6;
    //Elevator Motors
	public static final int elevatorMotor = 4;
	
	public static final int climberMotor1 = 9;
	public static final int climberMotor2 = 10;
	//PDP
	public static final int PDP = 8;
	//PCM
	public static final int PCM = 0;
	
	
}