package org.oastem.frc.robot;

import org.oastem.frc.control.TankDriveSystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
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
	private TalonSRX rightSlaveTalon;
	private TalonSRX leftSlaveTalon;
	
	private Talon liftingElevatorMotor; //change to Talon
	private Spark climbingElevatorMotor; //change to Talon
	
	private Spark intakeRightMotor;
	private Spark intakeLeftMotor;
	
	private Talon wristMotor;
	
	//ELEVATOR ENCODER
	private Encoder elevatorEnc;
	
	//BUTTONS
	private boolean intakeOut; //right bumper
	private boolean intakeIn; //left bumper
	
	private double elevatorUp; //right trigger
	private double elevatorDown; //left trigger
	
	private boolean wristFoldUp;//y button 
	private boolean wristFoldDown;//b button
	
	private boolean climb; //x button
	private boolean fastMode; //a button
	
	//private boolean elevatorMaxDown;//a button
	//private boolean elevatorMaxDownToggle; //toggle helper
	
	//SENSORS 
	private DigitalInput elevatorMinLimit; //limit switch on bottom of elevator
	private DigitalInput elevatorMaxLimit; 
	private DigitalInput articulationMinLimit; 
	
	//AUTONOMOUS CHOOSER 
	int autonomousCase = 0;
	final String pos0Auto = "Pos0";
	final String pos2Auto = "Pos2";
	final String goStraightAuto = "Straight";
	final String testAuto = "Test";
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
	
	//PID CONTROLLERS
	PIDController straightPID = new PIDController(0.15, 0.16, 0.0001, 100, 400);
	PIDController turnPID =   new PIDController(1.25, 1.25, 0.0001, 500, 100);
	//PIDController straightPID = new PIDController(0.13, 0.13, 0.0005, 100, 600);
	//PIDController turnPID =   new PIDController(1.3, 1.3, 0.0001, 500, 100);
	
	//AUTO HELPERS
	int state = 0; 
	double endOfArtTime; 
	double endOfStraightTime;
	double endOfLongStraightTime; 
	double endOfTurnTime; 
	double ejectCubeTime;
	
	public Robot() {
		//TANK DRIVE
		tankDrive.initializeTankDrive(C.Port.RIGHT_MASTER_DRIVE, C.Port.LEFT_MASTER_DRIVE,
									  C.Port.RIGHT_SLAVE_DRIVE, C.Port.LEFT_SLAVE_DRIVE);
		//GAME PAD
		pad = new LogitechGamingPad(0);
		
		//MOTORS
		rightMasterTalon = tankDrive.getRightMasterDrive();
		leftMasterTalon = tankDrive.getLeftMasterDrive();
		rightSlaveTalon = tankDrive.getRightSlaveDrive();
		leftSlaveTalon = tankDrive.getLeftSlaveDrive();
		
		liftingElevatorMotor= new Talon(C.Port.ELEVATOR_PORT);
		climbingElevatorMotor = new Spark(C.Port.ELEVATOR_2_PORT);
		
		intakeRightMotor = new Spark(C.Port.INTAKE_PORT);
		intakeLeftMotor = new Spark(C.Port.INTAKE_2_PORT);
		
		wristMotor = new Talon(C.Port.WRIST_PORT);
		
		elevatorMinLimit = new DigitalInput(C.Port.ELEVATOR_MIN_PORT);	
		elevatorMaxLimit = new DigitalInput(C.Port.ELEVATOR_MAX_PORT);
		articulationMinLimit = new DigitalInput(C.Port.ARTICULATION_MIN_PORT);
		
		//ENCODER
		elevatorEnc = new Encoder(C.Port.ELEVATOR_ENC_A, C.Port.ELEVATOR_ENC_B);
		
		//AUTONOMOUS CHOOSER 
		chooser = new SendableChooser<String>();
		chooser.addDefault("Position 0",  pos0Auto);
		chooser.addObject("Position 2", pos2Auto);
		chooser.addObject("Go Straight", goStraightAuto);
		chooser.addObject("Test", testAuto);
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
		prefs.putInt("Tolerable Error Left", 100); //encoder counts
		prefs.putInt("Tolerable Error Right", 100);
		prefs.putDouble("End of Articulation", 2);
		prefs.putDouble("End of Straight", 5);
		prefs.putDouble("End of Lift", 7);
		prefs.putDouble("End of Turn", 8);
		prefs.putDouble("Eject Cube", 10);
		
		endOfArtTime = prefs.getDouble("End of Articulation", 2);
		endOfStraightTime = prefs.getDouble("End of Straight", 5);
		endOfLongStraightTime = prefs.getDouble("End of Lift", 7);
		endOfTurnTime = prefs.getDouble("End of Turn", 8);
		ejectCubeTime = prefs.getDouble("Eject Cube", 10);
		
		//CAMERA
		server = CameraServer.getInstance();
		usbCamera = new UsbCamera("usbCamera",0);
		usbCamera.setResolution(720, 1280);
		usbCamera.setFPS(60);
		server.startAutomaticCapture(usbCamera);
		
		timer = new Timer();
	}

	public void autonomousInit() { 
		state = 0; 
		initDriveConstants();
		elevatorEnc.reset();
		
		autoSelected = (String)chooser.getSelected();
		autoSelected2 = (String)chooser2.getSelected();
		
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (autoSelected.equals(noAuto))
			noAutoCase();
		
		timer.start();
		tankDrive.drive(0, 0);
	}

	public void autonomousPeriodic() {
		if (timer.get() < 0.8) {
			wristMotor.set(1);
		}
		else if (timer.get() > 0.8) {
			wristMotor.set(0);
		}
		//C.Distances.AFTER_TURN_DIST
		if (autoSelected.equals(pos0Auto)){
			if (gameData.substring(0,2).equals("RL")) {
				straightTurnStraight(C.Distances.SCALE_DIST, 0, 0, 15, 15);
			}
			else if (gameData.substring(0,2).equals("LR")) {
				straightTurnStraight(C.Distances.SWITCH_DIST, C.Distances.TURN_DIST, C.Distances.AFTER_TURN_DIST, endOfStraightTime, endOfTurnTime);
				liftElevator();
				ejectCube();
			}
			else if (gameData.substring(0,2).equals("LL")) {
				if (autoSelected2.equals("Switch")) {
					straightTurnStraight(C.Distances.SWITCH_DIST, C.Distances.TURN_DIST, C.Distances.AFTER_TURN_DIST, endOfStraightTime, endOfTurnTime);
					liftElevator();
					ejectCube();
				}
				else if (autoSelected2.equals("Scale")) {
					straightTurnStraight(C.Distances.SCALE_DIST, 0, 0, 15, 15);
				}
			}
			else if (gameData.substring(0,2).equals("RR"))
				straightTurnStraight(C.Distances.IN_BETWEEN_DIST, C.Distances.TURN_DIST, 0, endOfLongStraightTime, endOfTurnTime);
		}
		
		else if (autoSelected.equals(pos2Auto)) {
			if (gameData.substring(0,2).equals("RL")) {
				straightTurnStraight(C.Distances.SWITCH_DIST, -C.Distances.TURN_DIST, C.Distances.AFTER_TURN_DIST, endOfStraightTime, endOfTurnTime);
				liftElevator();
				ejectCube();
			}
			else if (gameData.substring(0,2).equals("LR")) {
				straightTurnStraight(C.Distances.SCALE_DIST, 0, 0, 15, 15);
			}
			else if (gameData.substring(0,2).equals("RR")) {
				if (autoSelected2.equals("Switch")) {
					straightTurnStraight(C.Distances.SWITCH_DIST, -C.Distances.TURN_DIST, C.Distances.AFTER_TURN_DIST, endOfStraightTime, endOfTurnTime);
					liftElevator();
					ejectCube();
				}
				else if (autoSelected2.equals("Scale")) {
					straightTurnStraight(C.Distances.SCALE_DIST, 0, 0, 15, 15);
				}
			}
			else if (gameData.substring(0,2).equals("LL"))
				straightTurnStraight(C.Distances.IN_BETWEEN_DIST, -C.Distances.TURN_DIST, 0, endOfLongStraightTime, endOfTurnTime);
		}
		
		else if (autoSelected.equals(testAuto)) {
			straightTurnStraight(C.Distances.SWITCH_DIST, -C.Distances.TURN_DIST, C.Distances.AFTER_TURN_DIST, endOfStraightTime, endOfTurnTime);
			liftElevator();
			ejectCube();
		}
		
		else if (autoSelected.equals(goStraightAuto)) {
			straightTurnStraight(C.Distances.IN_BETWEEN_DIST, 0, 0, 15, 15);
		}
	}

	public void teleopInit() {
		tankDrive.drive(0, 0);
		
		leftMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		rightMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		
		elevatorEnc.reset();
		
		timer.start();
		secs = 150;
	}
	
	public void teleopPeriodic() {
		//BUTTONS
		intakeOut = pad.getRightBumper();
		intakeIn = pad.getLeftBumper();
		
		elevatorUp = pad.getRightTriggerValue();
		elevatorDown = pad.getLeftTriggerValue();
		
		wristFoldUp = pad.getYButton();
		wristFoldDown = pad.getBButton();
		
		//elevatorMaxDown = pad.getAButton();
		
		fastMode = pad.getAButton();
		climb = pad.getXButton();
		
		//DRIVING
		tankDrive.drive(-pad.getLeftAnalogY(), pad.getLeftAnalogY());
		
		//ELEVATOR
		//all the !_____ ensures that if the driver presses two opposing commands, nothing bad happens 
		if (pad.getRightTrigger() && !pad.getLeftTrigger() && !climb && elevatorMaxLimit.get()/*elevatorEnc.get() < C.Distances.MAX_ENC_COUNT && elevatorMaxLimit.get() && !elevatorMaxDownToggle*/) { //works
			liftingElevatorMotor.set(-elevatorUp);
		}
		
		if (pad.getLeftTrigger() && !pad.getRightTrigger() && !climb && elevatorMinLimit.get() /*&& !elevatorMaxDownToggle*/) { //works
			liftingElevatorMotor.set(elevatorDown);
		}
		
		//climbs up --> uses both elevator motors 
		if (climb && !pad.getLeftTrigger() && !pad.getRightTrigger() && elevatorMaxLimit.get()/*elevatorEnc.get() < C.Distances.MAX_ENC_COUNT && !elevatorMaxDownToggle*/) { 
				liftingElevatorMotor.set(1); 
				climbingElevatorMotor.set(1);
		}
		
		//elevator goes max down --> canceled if D-pad is pressed 
		/*if (!climb && !pad.getLeftTrigger() && !pad.getRightTrigger()) {
			if (elevatorMaxDown)
				elevatorMaxDownToggle = true;
			if (!elevatorMinLimit.get()|| (pad.getDPad() >= 0 && pad.getDPad() <=7))
				elevatorMaxDownToggle = false;
			if (elevatorMaxDownToggle)
				liftingElevatorMotor.set(-0.5);
			else if (!elevatorMaxDownToggle)
				liftingElevatorMotor.set(0);
		}*/

		//elevator doesn't move 
		if (!climb && !pad.getRightTrigger() && !pad.getLeftTrigger() /*&& !elevatorMaxDownToggle*/ || (pad.getLeftTrigger() && !elevatorMinLimit.get()) || (pad.getRightTrigger() && !elevatorMaxLimit.get()) || (climb && !elevatorMinLimit.get())) {
			liftingElevatorMotor.set(0);//go down until limit switch
			climbingElevatorMotor.set(0);
		}
		
		//OPERATING INTAKE 
		if (intakeIn) { //open intake 
			intakeRightMotor.set(0.6);
			intakeLeftMotor.set(0.6);
		}
		else if (intakeOut && !fastMode) { //close intake 
			intakeRightMotor.set(-0.4);
			intakeLeftMotor.set(-0.4);
		}
		else if (intakeOut && fastMode) { //close intake 
			intakeRightMotor.set(-1);
			intakeLeftMotor.set(-1);
		}
		else if (!intakeIn && !intakeOut) {
			intakeRightMotor.set(0);
			intakeLeftMotor.set(0);
		}
		
		//WRIST ARTICULATION 
		if (wristFoldDown && articulationMinLimit.get()) { //wrist closes 
			wristMotor.set(1);
		}
		else if (wristFoldUp) { //wrist opens 
			wristMotor.set(-1);
		}
		else if (!wristFoldDown && !wristFoldUp || (wristFoldDown && !articulationMinLimit.get())) {
			wristMotor.set(0);
		}
		
		printEncoderValues();
	}
	
	public void initDriveConstants() {
		rightMasterTalon.setIntegralAccumulator(0, 0, 10);
		rightMasterTalon.configAllowableClosedloopError(0, prefs.getInt("Tolerable Error Right", 20), 10);
		rightMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		rightMasterTalon.configClosedloopRamp(3, 10);
		
		leftMasterTalon.setIntegralAccumulator(0, 0, 10);
		leftMasterTalon.configAllowableClosedloopError(0, prefs.getInt("Tolerable Error Left", 20), 10);
		leftMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		leftMasterTalon.configClosedloopRamp(3, 10);
	}
	
	public void noAutoCase() {
		tankDrive.drive(0, 0);
	}
	
	public void straightTurnStraight(int straight, int turn, int straight2, double time, double time2) {
		if (state == 0) {
			tankDrive.configPID(straightPID);
			leftMasterTalon.set(ControlMode.Position, straight);
			rightMasterTalon.set(ControlMode.Position, -straight);
			if (timer.get() > time) {
				disableTalons();
				state = 1; 
			}
		}
		else if (state == 1){
			tankDrive.configPID(turnPID);
			rightMasterTalon.set(ControlMode.Position, turn);
			leftMasterTalon.set(ControlMode.Position, turn);
			if (timer.get() > time2) {
				disableTalons();
				state = 2; 
			}
		}
		else if (state == 2){
			tankDrive.configPID(straightPID);
			rightMasterTalon.set(ControlMode.Position, -straight2);
			leftMasterTalon.set(ControlMode.Position, straight2);
		}
		
		printEncoderValues();
	}
	
	public void ejectCube(){
		if (timer.get() > ejectCubeTime){
			intakeRightMotor.set(-0.6);
			intakeLeftMotor.set(-0.6);
		}
	}
	
	public void liftElevator(){
		if (timer.get() > endOfStraightTime && elevatorEnc.get() < 1300 )
			liftingElevatorMotor.set(-1);
		if (elevatorEnc.get() > 1300)
			liftingElevatorMotor.set(0);
	}
	
	public void disableTalons() {
		leftMasterTalon.set(ControlMode.Disabled, 0);
		rightMasterTalon.set(ControlMode.Disabled, 0);
		rightMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		leftMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
	}
	
	public void printEncoderValues() {
		SmartDashboard.putNumber("Left Master Encoder", leftMasterTalon.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Right Master Encoder", rightMasterTalon.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Left Slave Encoder", leftSlaveTalon.getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Right Slave Encoder", rightSlaveTalon.getSensorCollection().getQuadraturePosition());
		
		SmartDashboard.putNumber("Gyroscope Value", tankDrive.getAngle());
		SmartDashboard.putNumber("Elevator Encoder", elevatorEnc.get());
		
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
		SmartDashboard.putNumber("Timer: ", timer.get());
		SmartDashboard.putBoolean("Minimum Articulation Limit Switch", !articulationMinLimit.get());
		SmartDashboard.putBoolean("Minimum Elevator Limit Switch", !elevatorMinLimit.get());

	}

}