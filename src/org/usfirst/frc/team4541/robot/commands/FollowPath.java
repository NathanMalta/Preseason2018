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
	}
	
	@Override
	protected void initialize() {
		Robot.drivetrain.setLeftVel(0);
		Robot.drivetrain.setRightVel(0);
		Robot.state.zero();
	}
	
	public Path getPathFromType(PATH_TYPE pathType) {
		switch (pathType) {
		case TEST_PATH: {
			path = new Path();
			Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 18, 0);
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
				Robot.state.getHeading(), Robot.drivetrain.getRightVel(), -Robot.drivetrain.getLeftVel());
		RobotCmd cmd = this.path.update(latestPos);
		
		//debug print: lTarget, rTarget, lActualVel, rActualVel, RobotPosition
		System.out.println(Robot.drivetrain.leftMotor1.getClosedLoopTarget(0) +  "," + Robot.drivetrain.rightMotor1.getClosedLoopTarget(0) + "," + Robot.drivetrain.leftMotor1.getSelectedSensorVelocity(0) + "," + Robot.drivetrain.rightMotor1.getSelectedSensorVelocity(0) + "," + Robot.state.getPosition());
		
//		Robot.drivetrain.setLeftVel(cmd.getLeftVel());
//		Robot.drivetrain.setRightVel(cmd.getRightVel());
		
		Robot.drivetrain.setLeftVel(cmd.getRightVel());
		Robot.drivetrain.setRightVel(-cmd.getLeftVel());
		
//		System.out.println(Robot.state.getPosition() + " , " + Robot.state.getHeading());
	}
	
	@Override
	protected boolean isFinished() {
		return this.path.isFinished();
	}
	
	@Override
	protected void end() {
		Robot.drivetrain.setLeftVel(0);
		Robot.drivetrain.setRightVel(0);
		System.out.println("FINISHED PATH");
	}

}
