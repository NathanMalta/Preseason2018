package org.usfirst.frc.team4541.robot.commands;

import org.usfirst.frc.team4541.lib.ArcSegment;
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

	public static enum PATH_TYPE {
		TEST_PATH, TEST_PATH_CURVE, TEST_PATH_THICC_CURVE
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
			Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 60, 0);
			path.addSegment(seg1);
			return path;
		} case TEST_PATH_CURVE: {
			path = new Path();
			Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 60, 60);
			path.addSegment(seg1);
			Segment seg2 = new ArcSegment(new Point(60, 0), new Point(90, -30), new Point(60, -30), 60);
			path.addSegment(seg2);
			Segment seg3 = new ArcSegment(new Point(90, -30), new Point(120, -60), new Point(120, -30), 60);
			path.addSegment(seg3);
			Segment seg4 = new LineSegment(new Point(120, -60), new Point(170, -60), 60, 0);
			path.addSegment(seg4);
			return path;
		} case TEST_PATH_THICC_CURVE: {
			path = new Path();
			Segment seg1 = new LineSegment(new Point(0, 0), new Point(90, 0), 60);
			path.addSegment(seg1);
			Segment seg2 = new ArcSegment(new Point(90, 0), new Point(140, -50), new Point(90, -50), 60, 0);
			path.addSegment(seg2);
			return path;
		}
		default: {
			return null;
		}
		}
	}
	
	@Override
	public void execute() {
//		RobotPos latestPos = new RobotPos(Robot.state.getPosition(),
//				Robot.state.getHeading(), Robot.drivetrain.getRightVel(), Robot.drivetrain.getLeftVel()); //right way
		RobotPos latestPos = new RobotPos(Robot.state.getPosition(),
		Robot.state.getHeading(), Robot.drivetrain.getLeftVel(), Robot.drivetrain.getRightVel()); //opp way
		RobotCmd cmd = this.path.update(latestPos);
		
		//debug print: lTarget, rTarget, lActualVel, rActualVel, RobotPosition
//		System.out.println(Robot.drivetrain.leftMotor1.getClosedLoopTarget(0) +  "," + Robot.drivetrain.rightMotor1.getClosedLoopTarget(0) + "," + Robot.drivetrain.leftMotor1.getSelectedSensorVelocity(0) + "," + Robot.drivetrain.rightMotor1.getSelectedSensorVelocity(0) + "," + Robot.state.getPosition());
		System.out.println(Robot.drivetrain.getVel());
		
		Robot.drivetrain.setLeftVel(cmd.getLeftVel());
		Robot.drivetrain.setRightVel(cmd.getRightVel());
	}
	
	@Override
	protected boolean isFinished() {
		return this.path.isFinished();
	}
	
	@Override
	protected void end() {
		Robot.drivetrain.setLeftVel(0);
		Robot.drivetrain.setRightVel(0);
		if (this.isFinished()) {
			System.out.println("FINISHED PATH");
			System.out.println(Robot.state.getPosition());
		}
	}

}
