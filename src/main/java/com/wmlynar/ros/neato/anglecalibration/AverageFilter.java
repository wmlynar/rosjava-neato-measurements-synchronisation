package com.wmlynar.ros.neato.anglecalibration;

public class AverageFilter {
	private int count = 0;
	private double sum = 0;
	
	double array[] = new double[20];
	int index = 0;
	
	public synchronized void reset() {
		count = 0;
		sum = 0;
	}
	
	public synchronized void add(double d) {
		count++;
		sum+=d;
		
		if(count>20) {
			count = 100;
			sum *= 100./101;
		}
		array[index++]=d;
		if(index>=20) {
			index=0;
		}
	}
	
	public synchronized double get() {
//		return sum / count;
		double value = 0;
		for(int i=0; i<20; i++) {
			value += array[i];
		}
		return value / 20;
	}

}
