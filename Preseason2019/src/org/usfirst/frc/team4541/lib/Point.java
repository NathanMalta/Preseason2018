package org.usfirst.frc.team4541.lib;

public class Point {
	
	private double x = 0;
	private double y = 0;
	
	public Point(double xCord, double yCord) {
		this.x = xCord;
		this.y = yCord;
	}
	
	public double getX() {
		return x;
	}
	
	public double getY() {
		return y;
	}
	
	/*
	 * Get the distance between the origin and the point
	 */
	public double getHypot() {
		return Math.hypot(this.getX(), this.getY());
	}
	
	@Override
	public String toString() {
		return "(" + this.getX() + ", " + this.getY() + ")";
	}
	
	/*
	 * Take the cross product between two points
	 * 
	 * @param a the first point
	 * @param b the second point
	 */
	public static double cross(Point a, Point b) {
        return a.getX() * b.getY() - a.getY() * b.getX();
    }
	
	/*
	 * Create a point which represents the delta between
	 * two points
	 * 
	 * @param a the first point
	 * @param b the second point
	 */
	public static Point getDelta(Point a, Point b) {
		return new Point(b.getX() - a.getX(), b.getY() - a.getY());
	}
}
