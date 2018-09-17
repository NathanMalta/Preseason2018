package org.usfirst.frc.team4541.robot.commands;

import org.usfirst.frc.team4541.lib.LineSegment;
import org.usfirst.frc.team4541.lib.Path;
import org.usfirst.frc.team4541.lib.Point;
import org.usfirst.frc.team4541.lib.RobotCmd;
import org.usfirst.frc.team4541.lib.RobotPos;
import org.usfirst.frc.team4541.lib.Segment;
import org.usfirst.frc.team4541.robot.Constants;
import org.usfirst.frc.team4541.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

public class FollowPath extends Command {
	public Path path;
//	PIDController rPID;
//	PIDController lPID;
	public static enum PATH_TYPE {
		TEST_PATH
	}
	
	public FollowPath(PATH_TYPE pathType) {
		this.path = this.getPathFromType(pathType);
		requires(Robot.drivetrain);
		
		//TODO: tune velocity PID Controllers
		
//		rPID = new PIDController(0, 0, 0, 0, new PIDSource() {
//			@Override
//			public void setPIDSourceType(PIDSourceType pidSource) {
//			}
//			@Override
//			public PIDSourceType getPIDSourceType() {
//				return PIDSourceType.kDisplacement;
//			}
//			@Override
//			public double pidGet() {
//				return Robot.drivetrain.getRightVel();
//			}
//			
//		}, new PIDOutput() {
//			@Override
//			public void pidWrite(double output) {
//				Robot.drivetrain.rightMotor1.set(ControlMode.PercentOutput, output);
//				Robot.drivetrain.rightMotor1.set(ControlMode.PercentOutput, output);
//			}
//		});
//		
//		lPID = new PIDController(0, 0, 0, 0, new PIDSource() {
//			@Override
//			public void setPIDSourceType(PIDSourceType pidSource) {
//			}
//			@Override
//			public PIDSourceType getPIDSourceType() {
//				return PIDSourceType.kDisplacement;
//			}
//			@Override
//			public double pidGet() {
//				return Robot.drivetrain.getLeftVel();
//			}
//			
//		}, new PIDOutput() {
//			@Override
//			public void pidWrite(double output) {
//				Robot.drivetrain.leftMotor1.set(ControlMode.PercentOutput, output);
//				Robot.drivetrain.leftMotor2.set(ControlMode.PercentOutput, output);
//			}
//		});
//		lPID.setInputRange(-Constants.kMaxVelocity, Constants.kMaxVelocity); //objective velocity as input
//		lPID.setOutputRange(-1, 1);
//		
//		rPID.setInputRange(-Constants.kMaxVelocity, Constants.kMaxVelocity); //objective velocity as input
//		rPID.setOutputRange(-1, 1);
	}
	
	public Path getPathFromType(PATH_TYPE pathType) {
		switch (pathType) {
		case TEST_PATH: {
			path = new Path();
			Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 24, 0);
			path.addSegment(seg1);
			return path;
		} default: {
			return null;
		}
		}
	}
	
	@Override
	public void execute() {
		RobotPos latestPos = new RobotPos(Robot.state.getPosition(),
				Math.toRadians(Robot.gyro.getYaw()), Robot.drivetrain.getRightVel(), Robot.drivetrain.getLeftVel());
		RobotCmd cmd = this.path.update(latestPos);
//		rPID.setSetpoint(cmd.getRightVel());
//		lPID.setSetpoint(cmd.getLeftVel());
		Robot.drivetrain.setLeftVel(cmd.getLeftVel());
		Robot.drivetrain.setRightVel(cmd.getRightVel());
	}
	
	@Override
	protected boolean isFinished() {
		return this.path.isFinished();
	}

}
