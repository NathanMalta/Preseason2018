/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4541.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team4541.lib.LineSegment;
import org.usfirst.frc.team4541.lib.Path;
import org.usfirst.frc.team4541.lib.Point;
import org.usfirst.frc.team4541.lib.RobotCmd;
import org.usfirst.frc.team4541.lib.RobotPos;
import org.usfirst.frc.team4541.lib.Segment;
import org.usfirst.frc.team4541.profiling.RobotState;


import org.usfirst.frc.team4541.robot.Constants;

import org.usfirst.frc.team4541.robot.OI.TRIG_MODE;
import org.usfirst.frc.team4541.robot.commands.FollowPath;
import org.usfirst.frc.team4541.robot.commands.PathGroup;
import org.usfirst.frc.team4541.robot.subsystems.DriveTrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import org.usfirst.frc.team4541.robot.commands.FollowPath.PATH_TYPE;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
	public static OI oi;
	// Robot wide sensors
	public static AHRS gyro;
	
	// subsystems

	public static DriveTrain drivetrain;

	public static RobotState state;
	public WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(RobotMap.elevatorMotor);
	
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		gyro = new AHRS(SPI.Port.kMXP);
		drivetrain = new DriveTrain();
		oi = new OI();
		state = new RobotState(0, 0, 0, drivetrain.getRightPos(), drivetrain.getLeftPos());

		UsbCamera cam0 = CameraServer.getInstance().startAutomaticCapture(0);

		cam0.setWhiteBalanceAuto();
		cam0.setExposureManual(50);
		cam0.setFPS(20);
		cam0.setResolution(330, (int)(330*(9.0/16.0)));
		
		UsbCamera cam1 = CameraServer.getInstance().startAutomaticCapture(1);
		
		cam1.setWhiteBalanceAuto();
		cam1.setExposureManual(50);
		cam1.setFPS(20);
		cam1.setResolution(330, (int)(330*(9.0/16.0)));
	}

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {
		Scheduler.getInstance().removeAll();
		drivetrain.configTalons();
		state.end();
	}

	@Override
	public void disabledPeriodic() {
		System.out.println(elevatorMotor.getSelectedSensorPosition(0));
		Scheduler.getInstance().run();
	}


	// double currentTime = 0;
	// int counter = 0;
	@Override
	public void autonomousInit() {
		this.setPeriod(Constants.kDefaultDt);
		state.start();
		gyro.zeroYaw();
		Robot.state.zero();
		new PathGroup().start();
//		new FollowPath(PATH_TYPE.TEST_PATH_REVERSE).start();

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		state.end();
		gyro.setAngleAdjustment(0);
		drivetrain.leftMotor1.enableVoltageCompensation(true);
		drivetrain.leftMotor2.enableVoltageCompensation(true);
		drivetrain.rightMotor1.enableVoltageCompensation(true);
		drivetrain.rightMotor2.enableVoltageCompensation(true);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		//for checking if RobotState is correctly creating position
//		System.out.println(state.getPosition() + " , " + state.getHeading());
		//for tuning velocity PIDF - (mostly going to be I and F terms cause its velocity)
//		System.out.println(drivetrain.leftMotor1.getMotorOutputPercent() + "," + drivetrain.leftMotor1.getSelectedSensorVelocity(0)
//		+ "," + drivetrain.rightMotor1.getMotorOutputPercent() + "," + drivetrain.rightMotor1.getSelectedSensorVelocity(0));
				
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}

}