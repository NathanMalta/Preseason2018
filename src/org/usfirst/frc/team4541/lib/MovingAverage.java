package org.usfirst.frc.team4541.lib;

import java.util.ArrayList;

public class MovingAverage {
	int size;
	ArrayList<Double> queue;
	public MovingAverage(int size) {
		this.size = size;
		this.queue = new ArrayList<Double>();
	}
	
	public void add(double value) {
		queue.add(value);
		if (queue.size() > size) {
			queue.remove(0);
		}
	}
	
	public double getAvg() {
		double sum = 0;
		for (double val : this.queue) {
			sum += val;
		}
		return sum / this.queue.size();
	}
}
