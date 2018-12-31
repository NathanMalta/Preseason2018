/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Command;
import frc.lib.MathHelper;
import frc.robot.commands.PathGroup;
import frc.robot.subsystems.DriveTrain;

import com.kauailabs.navx.frc.AHRS;

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

	public static RobotPosEstimator estimator;
	
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		gyro = new AHRS(SPI.Port.kMXP);
		drivetrain = new DriveTrain();
		oi = new OI();
		estimator = new RobotPosEstimator(0, 0, 0, drivetrain.getRightPos(), drivetrain.getLeftPos());

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
		estimator.end();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}


	@Override
	public void autonomousInit() {
		estimator.start();
		gyro.zeroYaw();
		Robot.estimator.zero();

		Command forwardAndReverse = new PathGroup();
		forwardAndReverse.start();
		forwardAndReverse.close();
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
		estimator.end();
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
		Scheduler.getInstance().run();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	/**
	 * Get the current heading of the robot in radians
	 * @return the current robot heading in radians from -pi to pi
	 */
	public static double getAngleRad() {
		return MathHelper.angleToNegPiToPi(Math.toRadians(Robot.gyro.getAngle()));
	}

}