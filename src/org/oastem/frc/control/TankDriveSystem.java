package org.oastem.frc.control;

import org.oastem.frc.robot.PIDController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class TankDriveSystem {
	//TalonSRX's --> each side of tank drive has a master & slave motor 
	private TalonSRX rightMaster;
	private TalonSRX leftMaster; 
	private TalonSRX rightSlave; 
	private TalonSRX leftSlave;
	
	private ADXRS450_Gyro gyro;
	
	private static TankDriveSystem instance;

	public static TankDriveSystem getInstance() {
		if (instance == null) {
			instance = new TankDriveSystem();
		}
		return instance;
	}
	
	public void initializeTankDrive(int rMastPort, int lMastPort, int rSlavPort, int lSlavPort) {
		
		rightMaster = new TalonSRX(rMastPort);
		leftMaster = new TalonSRX(lMastPort);
		rightSlave = new TalonSRX(rSlavPort);
		leftSlave = new TalonSRX(lSlavPort);
		
		gyro = new ADXRS450_Gyro();
		
		initCan();
	}
	
	private void initCan() {
		FeedbackDevice encoder = FeedbackDevice.QuadEncoder;
		
		rightMaster.configSelectedFeedbackSensor(encoder, 0, 10); //timeout is 10 msec
		leftMaster.configSelectedFeedbackSensor(encoder, 0, 10); 
		
		if (rightSlave != null)
			rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());
		if (leftSlave != null) 
			leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
	}
	
	public void drive(double leftInput, double rightInput) {
		ControlMode mode = ControlMode.PercentOutput;
		
		rightMaster.set(mode, rightInput);
		leftMaster.set(mode, leftInput);
	}
	
	//Gyroscope & Accelerometer Information
	public double getAngle() {
		return gyro.getAngle();
	}

	public void calibrateGyro() {
		gyro.calibrate();
	}
	
	public void resetGyro() {
		gyro.reset();
	}
	
	//get talon's
	public TalonSRX getRightMasterDrive() {
		return rightMaster;
	}

	public TalonSRX getLeftMasterDrive() {
		return leftMaster;
	}

	public TalonSRX getRightSlaveDrive() {
		return rightSlave;
	}
	public TalonSRX getLeftSlaveDrive() {
		return leftSlave;
	}
	
	public void configPID(PIDController controller) {
		rightMaster.config_kP(0, controller.getRightPValue(), 10);
		rightMaster.config_kI(0, controller.getiValue(), 10);
		rightMaster.config_kD(0, controller.getdValue(), 10);
		rightMaster.config_IntegralZone(0, controller.getiZone(), 10);
		
		leftMaster.config_kP(0, controller.getLeftPValue(), 10);
		leftMaster.config_kI(0, controller.getiValue(), 10);
		leftMaster.config_kD(0, controller.getdValue(), 10);
		leftMaster.config_IntegralZone(0, controller.getiZone(), 10);
		
	}
}
