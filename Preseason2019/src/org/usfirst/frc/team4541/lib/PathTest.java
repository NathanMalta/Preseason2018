package org.usfirst.frc.team4541.lib;

public class PathTest {
	
	/*
	 * A class which helped test getClosestPointOnSegment with the help of Desmos Graphing Calculator
	 * https://www.desmos.com/calculator/qdsqwrsrke
	 */
	public static void main(String[] args) {
//		Segment testSegment = new LineSegment(new Point(-2.5, 6.7), new Point(-4.2, 10));
		Segment testSegment = new ArcSegment(new Point(0, 1), new Point(1, 0), new Point(0, 0));
		
		Point testPoint = new Point(0.5, 0.5);
		System.out.println("closest point:" + testSegment.getClosestPointOnSegment(testPoint));

	}

}
