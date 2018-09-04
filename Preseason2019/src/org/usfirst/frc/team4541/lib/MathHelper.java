package org.usfirst.frc.team4541.lib;

public class MathHelper {
	
	/*
	 * Checks if two doubles are approximately equal (equal to 3 decimal places)
	 * Used to circumvent floating point rounding errors from making == fail for
	 * equal values
	 * 
	 * @param v1: the first value to be compared
	 * @param v2: the second value to be compared
	 */
	public static boolean areApproxEqual(double v1, double v2) {
		return Math.round(v1 * 1000) == Math.round(v2 * 1000);
	}
}
