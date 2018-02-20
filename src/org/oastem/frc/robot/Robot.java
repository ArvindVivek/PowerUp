package org.oastem.frc.robot;

import org.oastem.frc.control.TankDriveSystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {
	//DRIVE SYSTEM
	private TankDriveSystem tankDrive = TankDriveSystem.getInstance();
	
	//GAMEPAD
	private LogitechGamingPad pad;
	
	//MOTORS
	private TalonSRX rightMasterTalon;
	private TalonSRX leftMasterTalon;
	
	private Spark liftingElevatorMotor;
	private Spark climbingElevatorMotor;
	
	private Spark intakeRightMotor;
	private Spark intakeLeftMotor;
	
	private Talon wristMotor;
	
	//BUTTONS
	private boolean intakeClose; //right bumper
	private boolean intakeOpen; //left bumper
	
	private double elevatorUp; //right trigger
	private double elevatorDown; //left trigger
	
	private boolean wristFoldUp;//y button 
	private boolean wristFoldDown;//b button
	
	private boolean elevatorMaxDown;//a button
	
	private boolean climb; //x button
	
	private boolean elevatorMaxDownToggle; //toggle helper
	
	//SENSORS 
	private DigitalInput elevatorMinLimit; //limit switch on bottom of elevator
	private DigitalInput articulationMinLimit; 
	
	//AUTONOMOUS CHOOSER 
	int autonomousCase = 0;
	final String pos0Auto = "Pos0";
	final String pos2Auto = "Pos2";
	final String noAuto = "None";
	SendableChooser<String> chooser;
	String autoSelected;
	
	int switchOrScaleCase = 0;
	final String switchCase = "Switch";
	final String scaleCase = "Scale";
	SendableChooser<String> chooser2;
	String autoSelected2;
	
	//DRIVER STATION MESSAGE
	String gameData = "";
	
	//PREFERENCES
	private Preferences prefs;
	
	//TIMER
	private Timer timer; 
	public int mins; // Andrew's Edit
	public int secs; // Andrew's Edit
	
	//CAMERA
	private CameraServer server;
	private UsbCamera usbCamera;
	
	//AUTO HELPER
	int state;
	boolean check;
	boolean check2;
	int scaleLength = 7400;
	int switchLength = 4800;
	int inBetween = 6400; 
	
	//PID CONTROLLERS
	PIDController straightPID = new PIDController(0.13, 0.13, 0.0005, 100, 600);
	PIDController turnPID =   new PIDController(1.3, 1.3, 0.0001, 500, 100);
	
	public Robot() {
		//TANK DRIVE
		tankDrive.initializeTankDrive(C.Port.RIGHT_MASTER_DRIVE, C.Port.LEFT_MASTER_DRIVE,
									  C.Port.RIGHT_SLAVE_DRIVE, C.Port.LEFT_SLAVE_DRIVE);
		//GAME PAD
		//pad = new LogitechGamingPad(0);
		
		//MOTORS
		rightMasterTalon = tankDrive.getRightMasterDrive();
		leftMasterTalon = tankDrive.getLeftMasterDrive();
		
		liftingElevatorMotor= new Spark(C.Port.ELEVATOR_PORT);
		climbingElevatorMotor = new Spark(C.Port.ELEVATOR_2_PORT);
		
		intakeRightMotor = new Spark(C.Port.INTAKE_PORT);
		intakeLeftMotor = new Spark(C.Port.INTAKE_2_PORT);
		
		wristMotor = new Talon(C.Port.WRIST_PORT);
		
		elevatorMinLimit = new DigitalInput(C.Port.ELEVATOR_MIN_PORT);
		articulationMinLimit = new DigitalInput(C.Port.ART_MIN_PORT);
		
		//AUTONOMOUS CHOOSER 
		chooser = new SendableChooser<String>();
		chooser.addDefault("Position 0",  pos0Auto);
		chooser.addObject("Position 2", pos2Auto);
		chooser.addObject("No Auto", noAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		//CHOOSER2
		chooser2 = new SendableChooser<String>();
		chooser2.addDefault("Switch",  switchCase);
		chooser2.addObject("Scale", scaleCase);
		SmartDashboard.putData("Scale or Switch", chooser2);
		
		//PREFERENCES
		prefs = Preferences.getInstance();
		
		prefs.remove(".type");
		prefs.putDouble("F-Gain Left", 0); 
		prefs.putDouble("F-Gain Right", 0);
		prefs.putDouble("P-Value Left", 1);
		prefs.putDouble("P-Value Right", 1);
		prefs.putDouble("I-Value Left", 0.0005);
		prefs.putDouble("I-Value Right", 0.0005);
		prefs.putDouble("D-Value Left", 200);
		prefs.putDouble("D-Value Right", 200);
		prefs.putInt("I-Zone Left", 500); //encoder counts
		prefs.putInt("I-Zone Right", 500);
		prefs.putInt("Tolerable Error Left", 100); //encoder counts
		prefs.putInt("Tolerable Error Right", 100);
		
		server = CameraServer.getInstance();
		usbCamera = new UsbCamera("usbCamera",0);
		usbCamera.setResolution(720, 1280);
		usbCamera.setFPS(60);
		server.startAutomaticCapture(usbCamera);
		
		timer = new Timer();
		state = 0;
		check = false;
	}

	public void autonomousInit() { 
		state = 0;
		check = false;
		check2 = false; 
		
		initDriveConstants();
		
		autoSelected = (String)chooser.getSelected();
		autoSelected2 = (String)chooser2.getSelected();
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (autoSelected.equals(noAuto))
			noAutoCase();
		
		timer.start();
		
		tankDrive.drive(0, 0);
	}

	public void autonomousPeriodic() {
		if (articulationMinLimit.get()) {
			wristMotor.set(1);
		}
		else if (!articulationMinLimit.get()) {
			wristMotor.set(0);
		}
		
		if (autoSelected.equals(pos0Auto)) {
			if (gameData.substring(0,2).equals("RL")) {
				goStraightTurn(scaleLength, 0);
				//raise elevator to max height
			}
			else if (gameData.substring(0,2).equals("LR")) {
				goStraightTurn(switchLength, 590);
				//raise elevator half way, deliver cube 
			}
			else if (gameData.substring(0,2).equals("LL")) {
				if (autoSelected2.equals("Switch")) {
					goStraightTurn(switchLength, 590);
					//raise elevator half way, deliver cube 
				}
				else if (autoSelected2.equals("Scale")) {
					goStraightTurn(scaleLength, 590);
					//raise elevator to max height
				}
			}
			else if (gameData.substring(0,2).equals("RR"))
				goStraightTurn(inBetween, 590);
		}
		
		else if (autoSelected.equals(pos2Auto)) {
			if (gameData.substring(0,2).equals("RL")) {
				goStraightTurn(switchLength, -590);
				//raise elevator half way, deliver cube 
			}
			else if (gameData.substring(0,2).equals("LR")) {
				goStraightTurn(scaleLength, 0);
				//raise elevator to max height
			}
			else if (gameData.substring(0,2).equals("RR")) {
				System.out.print("fail");
				if (autoSelected2.equals("Switch")) {
					goStraightTurn(switchLength, -590);
					//raise elevator half way, deliver cube 
				}
				else if (autoSelected2.equals("Scale")) {
					goStraightTurn(scaleLength, 0);
					//raise elevator to max height
				}
			}
			else if (gameData.substring(0,2).equals("LL"))
				goStraightTurn(inBetween, -590);
		}
		
	}

	public void teleopInit() {
		tankDrive.drive(0, 0);
		leftMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		rightMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		
		timer.start();
		secs = 150;
	}
	
	public void teleopPeriodic() {
		//BUTTONS
		intakeClose = pad.getRightBumper();
		intakeOpen = pad.getLeftBumper();
		
		elevatorUp = pad.getRightTriggerValue();
		elevatorDown = pad.getLeftTriggerValue();
		
		wristFoldUp = pad.getYButton();
		wristFoldDown = pad.getBButton();
		
		elevatorMaxDown = pad.getAButton();
		
		climb = pad.getXButton();
		
		//DRIVING
		tankDrive.drive(-pad.getLeftAnalogY(), pad.getRightAnalogY());
		
		//ELEVATOR
		//all the !_____ ensures that if the driver presses two opposing commands, nothing bad happens 
		if (pad.getRightTrigger() && !pad.getLeftTrigger() && !elevatorMaxDownToggle && !climb) { //works
			liftingElevatorMotor.set(-elevatorUp);
		}
		
		if (pad.getLeftTrigger() && !pad.getRightTrigger() && !elevatorMaxDownToggle && !climb) { //works
			liftingElevatorMotor.set(elevatorDown);
		}
		
		//climbs up --> uses both elevator motors 
		if (climb && !pad.getLeftTrigger() && !pad.getRightTrigger() && !elevatorMaxDownToggle) { 
				liftingElevatorMotor.set(0.8); 
				climbingElevatorMotor.set(0.8);
		}
		
		//elevator goes max down --> canceled if D-pad is pressed 
		if (!climb && !pad.getLeftTrigger() && !pad.getRightTrigger()) {
			if (elevatorMaxDown)
				elevatorMaxDownToggle = true;
			if (!elevatorMinLimit.get()|| (pad.getDPad() >= 0 && pad.getDPad() <=7))
				elevatorMaxDownToggle = false;
			if (elevatorMaxDownToggle)
				liftingElevatorMotor.set(-0.2);
			else if (!elevatorMaxDownToggle)
				liftingElevatorMotor.set(0);
		}
		
		//elevator doesn't move 
		if (!climb && !pad.getRightTrigger() && !pad.getLeftTrigger() && !elevatorMaxDownToggle) {
			liftingElevatorMotor.set(0);//go down until limit switch
			climbingElevatorMotor.set(0);
		}
		
		//OPERATING INTAKE 
		if (intakeOpen) { //open intake 
			intakeRightMotor.set(0.4);
			intakeLeftMotor.set(0.4);
		}
		else if (intakeClose) { //close intake 
			intakeRightMotor.set(-0.4);
			intakeLeftMotor.set(-0.4);
		}
		else if (!intakeOpen && !intakeClose) {
			intakeRightMotor.set(0);
			intakeLeftMotor.set(0);
		}
		
		//WRIST ARTICULATION 
		if (wristFoldDown) { //wrist closes 
			wristMotor.set(1);
		}
		else if (wristFoldUp) { //wrist opens 
			wristMotor.set(-1);
		}
		else if ((!wristFoldDown && !wristFoldUp) || !articulationMinLimit.get()) {
			wristMotor.set(0);
		}
		printEncoderValues();
		
	}
	
	public void initDriveConstants() {
		rightMasterTalon.setIntegralAccumulator(0, 0, 10);
		rightMasterTalon.configAllowableClosedloopError(0, prefs.getInt("Tolerable Error Right", 0), 10);
		rightMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		rightMasterTalon.configClosedloopRamp(3, 10);
		
		leftMasterTalon.setIntegralAccumulator(0, 0, 10);
		leftMasterTalon.configAllowableClosedloopError(0, prefs.getInt("Tolerable Error Left", 0), 10);
		leftMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		leftMasterTalon.configClosedloopRamp(3, 10);
	}
	
	public void noAutoCase() {
		rightMasterTalon.set(ControlMode.PercentOutput, 0);
		leftMasterTalon.set(ControlMode.PercentOutput, 0);
	}
	
	public void goStraightTurn(int straight, int turn) {
		if (state == 0) {
			if (timer.get() > 1 && timer.get() < 2) {
				check = true; 
			}
			tankDrive.configPID(straightPID);
			leftMasterTalon.set(ControlMode.Position, straight);
			rightMasterTalon.set(ControlMode.Position, -straight);
		}
		else if (state == 1){
			tankDrive.configPID(turnPID);
			rightMasterTalon.set(ControlMode.Position, turn);
			leftMasterTalon.set(ControlMode.Position, turn);
		}
		if (check && Math.abs(leftMasterTalon.getClosedLoopError(0)) < 100 &&
			Math.abs(rightMasterTalon.getClosedLoopError(0)) < 100 ) {
			disableTalons();
			state = 1; 
			check = false;
		}
		if (state == 1 && Math.abs(leftMasterTalon.getClosedLoopError(0)) < 30 &&
			Math.abs(rightMasterTalon.getClosedLoopError(0)) < 30) {
			disableTalons();
			state = 2;
		}
		
		printEncoderValues();
	}
	
	/*public void goStraightTurnGoStraight (int straight, int turn, int straight2) {
		if (state == 0) {
			if (timer.get() > 1 && timer.get() < 2) {
				check = true; 
			}
			tankDrive.configPID(straightPID);
			leftMasterTalon.set(ControlMode.Position, straight);
			rightMasterTalon.set(ControlMode.Position, -straight);
		}
		else if (state == 1){
			if (timer2.get() > 0.2 && timer2.get() < 0.3)
				check2 = true; 
			tankDrive.configPID(turnPID);
			leftMasterTalon.set(ControlMode.Position, turn);
			rightMasterTalon.set(ControlMode.Position, turn);
		}
		else if (state == 2){
			tankDrive.configPID(straightPID);
			leftMasterTalon.set(ControlMode.Position, straight2);
			rightMasterTalon.set(ControlMode.Position, -straight2);
		}
		if (check && Math.abs(leftMasterTalon.getClosedLoopError(0)) < 100 &&
			Math.abs(rightMasterTalon.getClosedLoopError(0)) < 100 ) {
			disableTalons();
			check = false;
			state = 1;
			timer2.start();
		}
		if (check2 && Math.abs(leftMasterTalon.getClosedLoopError(0)) < 20 &&
			Math.abs(rightMasterTalon.getClosedLoopError(0)) < 20) {
			disableTalons();
			check2 = false; 
			state = 2; 
		}
		if (state == 2 && Math.abs(leftMasterTalon.getClosedLoopError(0)) < 20 &&
			Math.abs(rightMasterTalon.getClosedLoopError(0)) < 20) {
			disableTalons();
			state = 3; 
		}
		printEncoderValues();
	}
	*/
	public void disableTalons() {
		leftMasterTalon.set(ControlMode.Disabled, 0);
		rightMasterTalon.set(ControlMode.Disabled, 0);
		rightMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		leftMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
	}
	
	public void printEncoderValues() {
				
		SmartDashboard.putNumber("Gyroscope Value", tankDrive.getAngle());
				
		// Andrew's Edit
		SmartDashboard.putNumber( "Left Analog Y", -pad.getLeftAnalogY() );
		SmartDashboard.putNumber("Right Analog", -pad.getRightAnalogY() );
		SmartDashboard.putBoolean("Ay", pad.getAButton() );
		SmartDashboard.putBoolean("B", pad.getBButton() );
		SmartDashboard.putBoolean("Y", pad.getYButton() );
		SmartDashboard.putBoolean("X", pad.getXButton() );
		SmartDashboard.putBoolean("Intake Open", pad.getLeftBumper() );
		SmartDashboard.putBoolean("Intake Close", pad.getRightBumper() );
		
		secs = (int)(150 - timer.get() );
		mins = (secs/60);
		
		SmartDashboard.putString("Actual Timer", mins + ":" + String.format("%02d", secs%60) );
		SmartDashboard.putNumber("Raw Timer", secs);
		
		SmartDashboard.putBoolean("Climbing", pad.getStartButton() );
		
		SmartDashboard.putBoolean("Elevator Down", pad.getLeftTrigger() );
		SmartDashboard.putBoolean("Elevator Up", pad.getRightTrigger() );
		SmartDashboard.putNumber("Left Speed: ", pad.getLeftTriggerValue() );
		SmartDashboard.putNumber("Right Speed", pad.getRightTriggerValue() );

		SmartDashboard.putString("State", state + "");
		SmartDashboard.putNumber("Error Right", Math.abs(rightMasterTalon.getClosedLoopError(0)));
		SmartDashboard.putNumber("Error Left", Math.abs(leftMasterTalon.getClosedLoopError(0)));
		SmartDashboard.putBoolean("Auto Check", check);
		SmartDashboard.putBoolean("Check2", check2);
		SmartDashboard.putNumber("Timer: ", timer.get());
	}

}