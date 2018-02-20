package org.oastem.frc.robot;

public class PIDController {
	private double leftPValue;
	private double rightPValue;
	private double iValue;
	private double dValue;
	private int iZone;
	
	public PIDController (double leftP, double rightP, 
						  double i, double d, int iZ) {
		setLeftPValue(leftP);
		setRightPValue(rightP);
		setiValue(i);
		setdValue(d);
		setiZone(iZ);	
	}

	public double getLeftPValue() {
		return leftPValue;
	}

	public void setLeftPValue(double leftPValue) {
		this.leftPValue = leftPValue;
	}

	public double getRightPValue() {
		return rightPValue;
	}

	public void setRightPValue(double rightPValue) {
		this.rightPValue = rightPValue;
	}

	public double getiValue() {
		return iValue;
	}

	public void setiValue(double iValue) {
		this.iValue = iValue;
	}

	public double getdValue() {
		return dValue;
	}

	public void setdValue(double dValue) {
		this.dValue = dValue;
	}

	public int getiZone() {
		return iZone;
	}

	public void setiZone(int iZone) {
		this.iZone = iZone;
	}

}