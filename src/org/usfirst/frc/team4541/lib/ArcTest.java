package org.usfirst.frc.team4541.lib;

import org.usfirst.frc.team4541.robot.Constants;
import org.usfirst.frc.team4541.robot.commands.FollowPath;

import edu.wpi.first.wpilibj.PIDController;

public class ArcTest {
	
	/*
	 * A class which helped test getClosestPointOnSegment with the help of Desmos Graphing Calculator
	 * https://www.desmos.com/calculator/qdsqwrsrke
	 */
	public static void main(String[] args) {
		RobotPos currentPos = new RobotPos(0,0, 0, 0,0);
		double turnRadius = 10;
		double currentVel = 20;
		
		ConnectionArc arc = new ConnectionArc(20, true);
		double lWheelVel = arc.getLeftVelocityTarget(currentVel);
		double rWheelVel = arc.getRightVelocityTarget(currentVel);
		
		while (true) {
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			RobotCmd cmd = new RobotCmd(lWheelVel, rWheelVel);
			
			double heading = currentPos.heading + ((cmd.getRightVel() - cmd.getLeftVel()) / Constants.kWheelBase) * Constants.kDefaultDt;
			
			double xPos = currentPos.position.getX() + (cmd.getLeftVel() + cmd.getRightVel())/2 * Constants.kDefaultDt * Math.cos(heading);
			double yPos = currentPos.position.getY() + (cmd.getLeftVel() + cmd.getRightVel())/2 * Constants.kDefaultDt * Math.sin(heading);
			currentPos = new RobotPos(xPos, yPos, heading, cmd.rVelDesired, cmd.lVelDesired);
			System.out.println(currentPos);
//			System.out.println((lWheelVel + rWheelVel) / 2);
		}
	}
	
	public static double ftToRad(double inches) {
		double circum = Constants.kWheelDiameter * Math.PI;
		return (inches / circum) * Math.PI * 2;
	}

}