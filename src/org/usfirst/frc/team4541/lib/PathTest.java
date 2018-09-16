package org.usfirst.frc.team4541.lib;

import org.usfirst.frc.team4541.robot.Constants;

import edu.wpi.first.wpilibj.PIDController;

public class PathTest {
	
	/*
	 * A class which helped test getClosestPointOnSegment with the help of Desmos Graphing Calculator
	 * https://www.desmos.com/calculator/qdsqwrsrke
	 */
	public static void main(String[] args) {
//		Segment testSegment = new LineSegment(new Point(-2.5, 6.7), new Point(-4.2, 10), 0);
//		Segment testSegment = new ArcSegment(new Point(2, 0), new Point(0, -2), new Point(0, 0), 0);
//		
//		Point testPoint = new Point(1, -1);
//		Point lookahead = testSegment.getClosestPointOnSegment(testPoint);
//		
//		System.out.println("closest point:" + lookahead);
//		System.out.println("dist 1:" + testSegment.getDistanceToEndpoint(testSegment.getStartPoint()));
//		System.out.println("dist 2:" + testSegment.getDistanceToEndpoint(lookahead));
//		
//		VelocityManager manager = new VelocityManager(Constants.kMaxAccel, Constants.kMaxJerk);
//		double distNeeded = manager.getDistanceNeededToAccel(Constants.kMaxAccel, 30, 0);
//		System.out.println(distNeeded);
		
		Path path = new Path();
		Segment seg1 = new LineSegment(new Point(0, 0), new Point(60, 0), 60, 36);
		path.addSegment(seg1);
		Segment seg2 = new ArcSegment(new Point(60, 0), new Point(90, 30), new Point(60, 30), 36, 24);
		path.addSegment(seg2);
		Segment seg3 = new LineSegment(new Point(90, 30), new Point(90, 80), 24, 12);
		path.addSegment(seg3);
		Segment seg4 = new ArcSegment(new Point(90, 80), new Point(110, 100), new Point(110, 80), 12);
		path.addSegment(seg4);
		Segment seg5 = new LineSegment(new Point(110, 100), new Point(150, 100), 24, 0);
		path.addSegment(seg5);
		
		RobotPos currentPos = new RobotPos(0,0, 0, 0,0);
		
		double pathTime = 0;
		while (!path.isFinished()) {
			try {
				Thread.sleep(1);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			RobotCmd cmd = path.update(currentPos);
//			double heading = currentPos.heading - ((Constants.kWheelDiameter/2) / Constants.kWheelBase) * ftToRad(cmd.getRightVel() - cmd.getLeftVel()) * path.manager.dt;
//			double heading = (currentPos.heading - currentPos.getDifferential() * path.manager.dt) / (Constants.kWheelDiameter / 2);
			
			double heading = currentPos.heading + ((cmd.getRightVel() - cmd.getLeftVel()) * 2 / Constants.kWheelBase) * path.manager.dt;
			
//			double dh = currentPos.heading - heading;
//			if (dh > 0.1) {
//				heading = currentPos.heading + 0.1;
//			} else if (dh < -0.1) {
//				heading = currentPos.heading - 0.1;
//			}
			
			
			double xPos = currentPos.position.getX() + (cmd.getLeftVel() + cmd.getRightVel())/2 * path.manager.dt * Math.cos(heading);
			double yPos = currentPos.position.getY() + (cmd.getLeftVel() + cmd.getRightVel())/2 * path.manager.dt * Math.sin(heading);
			currentPos = new RobotPos(xPos, yPos, heading, cmd.lVelDesired, cmd.rVelDesired);
			System.out.println(currentPos);
			pathTime += Constants.kDefaultDt;
		}
		System.out.println("Estimate time Required " + pathTime + " sec");
		
	}
	
	public static double ftToRad(double inches) {
		double circum = Constants.kWheelDiameter * Math.PI;
		return (inches / circum) * Math.PI * 2;
	}

}
