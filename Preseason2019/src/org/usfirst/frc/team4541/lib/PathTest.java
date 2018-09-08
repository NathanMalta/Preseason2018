package org.usfirst.frc.team4541.lib;

import org.usfirst.frc.team4541.robot.Constants;

public class PathTest {
	
	/*
	 * A class which helped test getClosestPointOnSegment with the help of Desmos Graphing Calculator
	 * https://www.desmos.com/calculator/qdsqwrsrke
	 */
	public static void main(String[] args) {
//		Segment testSegment = new LineSegment(new Point(-2.5, 6.7), new Point(-4.2, 10), 0);
		Segment testSegment = new ArcSegment(new Point(2, 0), new Point(0, -2), new Point(0, 0), 0);
		
		Point testPoint = new Point(1, -1);
		Point lookahead = testSegment.getClosestPointOnSegment(testPoint);
		
		System.out.println("closest point:" + lookahead);
		System.out.println("dist 1:" + testSegment.getDistanceToEndpoint(testSegment.getStartPoint()));
		System.out.println("dist 2:" + testSegment.getDistanceToEndpoint(lookahead));
		
		VelocityManager manager = new VelocityManager(Constants.kMaxAccel, Constants.kMaxJerk);
		double distNeeded = manager.getDistanceNeededToAccel(Constants.kMaxAccel, 30, 0);
		System.out.println(distNeeded);
	}

}
