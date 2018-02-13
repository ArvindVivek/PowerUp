package org.oastem.frc.robot;

import org.oastem.frc.control.TankDriveSystem;
import org.oastem.frc.motion.MotionProfileExample;
import org.oastem.frc.motion.SwitchLeft;
import org.oastem.frc.motion.SwitchRight;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

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
	
	private Talon wristRightMotor;
	private Talon wristLeftMotor;
	
	//BUTTONS
	private boolean intakeClose; //right bumper
	private boolean intakeOpen; //left bumper
	
	private double elevatorUp; //right trigger
	private double elevatorDown; //left trigger
	
	private boolean wristFoldUp;//y button 
	private boolean wristFoldDown;//a button
	
	private boolean elevatorMaxUp;//b button
	private boolean elevatorMaxDown; //x button
	
	private boolean climb; //start button
	
	private boolean elevatorMaxUpToggle; //toggle helper
	private boolean elevatorMaxDownToggle; //toggle helper
	
	//MOTION PROFILES
	private MotionProfileExample leftProfile;
	private MotionProfileExample rightProfile; 
	
	//SENSORS 
	private DigitalInput elevatorMinLimit; //limit switch on bottom of elevator
	private DigitalInput elevatorMaxLimit; //limit switch on top of elevator
	
	//AUTONOMOUS CHOOSER 
	int autonomousCase = 0;
	final String pos0Auto = "Pos0";
	final String pos1Auto = "Pos1";
	final String pos2Auto = "Pos2";
	final String noAuto = "None";
	SendableChooser<String> chooser;
	String autoSelected;
	
	//AUTO HELPERS
	boolean start = true;
	String gameData = "";
	
	//PREFERENCES
	private Preferences prefs;
	
	//TIMER
	private Timer timer; 
	public int mins; // Andrew's Edit
	public int secs; // Andrew's Edit
	
	public Robot() {
		//TANK DRIVE
		tankDrive.initializeTankDrive(C.Port.RIGHT_MASTER_DRIVE, C.Port.LEFT_MASTER_DRIVE,
									  C.Port.RIGHT_SLAVE_DRIVE, C.Port.LEFT_SLAVE_DRIVE);
		//GAME PAD
		pad = new LogitechGamingPad(0);
		
		//MOTORS
		rightMasterTalon = tankDrive.getRightMasterDrive();
		leftMasterTalon = tankDrive.getLeftMasterDrive();
		
		liftingElevatorMotor= new Spark(C.Port.ELEVATOR_PORT);
		climbingElevatorMotor = new Spark(C.Port.ELEVATOR_2_PORT);
		
		intakeRightMotor = new Spark(C.Port.INTAKE_PORT);
		intakeLeftMotor = new Spark(C.Port.INTAKE_2_PORT);
		
		wristRightMotor = new Talon(C.Port.WRIST_PORT);
		wristLeftMotor = new Talon(C.Port.WRIST_2_PORT);
		
		elevatorMinLimit = new DigitalInput(C.Port.ELEVATOR_MIN_PORT);
		elevatorMaxLimit = new DigitalInput(C.Port.ELEVATOR_MAX_PORT);	
		
		//AUTONOMOUS CHOOSER 
		chooser = new SendableChooser<String>();
		chooser.addDefault("Position 0",  pos0Auto);
		chooser.addObject("Position 1", pos1Auto);
		chooser.addObject("Position 2", pos2Auto);
		chooser.addObject("No Auto", noAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		//PREFERENCES
		prefs = Preferences.getInstance();
		
		prefs.remove(".type");
		prefs.putDouble("F-Gain Left", 0); 
		prefs.putDouble("F-Gain Right", 0);
		prefs.putDouble("P-Value Left", 0.5);
		prefs.putDouble("P-Value Right", 0.5);
		prefs.putDouble("I-Value Left", 0.001);
		prefs.putDouble("I-Value Right", 0.001);
		prefs.putDouble("D-Value Left", 0);
		prefs.putDouble("D-Value Right", 0);
		prefs.putInt("I-Zone Left", 100); //encoder counts
		prefs.putInt("I-Zone Right", 100);
		prefs.putInt("Tolerable Error Left", 20); //encoder counts
		prefs.putInt("Tolerable Error Right", 20);
		
		timer = new Timer();
	}

	public void autonomousInit() { 
		initDriveConstants();
		
		autoSelected = (String)chooser.getSelected();
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		
		if (autoSelected.equals(noAuto))
			noAutoCase();

		else if (autoSelected.equals(pos1Auto) && gameData.charAt(0) == 'L') 
			setMotionProfile(SwitchRight.getUpdatedPoints(), SwitchLeft.Points);
		
		else if (autoSelected.equals(pos1Auto) && gameData.charAt(0) == 'R') 
			setMotionProfile(SwitchLeft.getUpdatedPoints(), SwitchRight.Points);
		
		else if (autoSelected.equals(pos0Auto) || autoSelected.equals(pos2Auto)) {
			leftMasterTalon.set(ControlMode.Position, 0);
			rightMasterTalon.set(ControlMode.Position, 0);
		}
	}

	public void autonomousPeriodic() {
		if (autoSelected.equals(pos1Auto) && rightProfile.getIsLast() && leftProfile.getIsLast()) {
			if (!rightProfile.getIsLast() && !leftProfile.getIsLast()) {
				rightProfile.control();
				leftProfile.control();
			
				SetValueMotionProfile setOutputRight = rightProfile.getSetValue();
				SetValueMotionProfile setOutputLeft = leftProfile.getSetValue();
			
				rightMasterTalon.set(ControlMode.MotionProfile, setOutputRight.value);
				leftMasterTalon.set(ControlMode.MotionProfile, setOutputLeft.value);
			}
			else
				;//articulate, raise the elevator to switch height, drop off cube 
		}
		
		else if ((autoSelected.equals(pos0Auto) || autoSelected.equals(pos2Auto))&& rightProfile.getIsLast() && leftProfile.getIsLast()) {
			//if robot is done moving, articulate, raise the elevator to scale height 
		}
		printEncoderValues();
	}

	public void teleopInit() {
		tankDrive.drive(0, 0);
		
		//Andrew's Edit
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
		wristFoldDown = pad.getAButton();
		
		elevatorMaxUp = pad.getBButton();
		elevatorMaxDown = pad.getXButton();
		
		climb = pad.getStartButton();
		
		//DRIVING
		tankDrive.drive(-0.5*pad.getLeftAnalogY(), 0.5*pad.getRightAnalogY());
		
		//ELEVATOR
		//all the !_____ ensures that if the driver presses two opposing commands, nothing bad happens 
		if (pad.getRightTrigger() && !pad.getLeftTrigger() && !elevatorMaxUpToggle && !elevatorMaxDownToggle && !climb) { //works
			liftingElevatorMotor.set(elevatorUp);
		}
		
		if (pad.getLeftTrigger() && !pad.getRightTrigger() && !elevatorMaxUpToggle && !elevatorMaxDownToggle && !climb) { //works
			liftingElevatorMotor.set(-elevatorDown);
		}
		
		//climbs up --> uses both elevator motors 
		if (climb && !pad.getLeftTrigger() && !pad.getRightTrigger() && !elevatorMaxUpToggle && !elevatorMaxDownToggle) { 
			if (elevatorMaxLimit.get()) {
				liftingElevatorMotor.set(0.2); //goes up until limit switch is pressed 
				climbingElevatorMotor.set(0.2);
			}
			if (!elevatorMaxLimit.get()) {
				liftingElevatorMotor.set(0);
				climbingElevatorMotor.set(0);
			}	
		}
		
		//elevator goes max up --> canceled if D-pad is pressed 
		if (!elevatorMaxDownToggle && !climb && !pad.getLeftTrigger() && !pad.getRightTrigger()) {
			if (elevatorMaxUp)
				elevatorMaxUpToggle = true;
			if (!elevatorMaxLimit.get() || (pad.getDPad() >= 0 && pad.getDPad() <=7))
				elevatorMaxUpToggle = false;
			if (elevatorMaxUpToggle)
				liftingElevatorMotor.set(0.2);
			else if (!elevatorMaxUpToggle)
				liftingElevatorMotor.set(0);
		}
		
		//elevator goes max down --> canceled if D-pad is pressed 
		if (!elevatorMaxUpToggle && !climb && !pad.getLeftTrigger() && !pad.getRightTrigger()) {
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
		if (!climb && !pad.getRightTrigger() && !pad.getLeftTrigger() && !elevatorMaxUpToggle && !elevatorMaxDownToggle) {
			liftingElevatorMotor.set(0);//go down until limit switch
			climbingElevatorMotor.set(0);
		}
		
		//OPERATING INTAKE 
		if (intakeOpen) { //open intake 
			intakeRightMotor.set(0.2);
			intakeLeftMotor.set(-0.2);
		}
		else if (intakeClose) { //close intake 
			intakeRightMotor.set(-0.2);
			intakeLeftMotor.set(0.2);
		}
		else if (!intakeOpen && !intakeClose) {
			intakeRightMotor.set(0);
			intakeLeftMotor.set(0);
		}
		
		//WRIST ARTICULATION 
		if (wristFoldDown) { //wrist closes 
			wristRightMotor.set(0.2);
			wristLeftMotor.set(0.2);
		}
		else if (wristFoldUp) { //wrist opens 
			wristRightMotor.set(-0.2);
			wristLeftMotor.set(-0.2);
		}
		else if (!wristFoldDown && !wristFoldUp) {
			wristRightMotor.set(0);
			wristLeftMotor.set(0);
		}
		
		printEncoderValues();
	}
	
	public void initDriveConstants() {
		rightMasterTalon.config_kP(0, prefs.getDouble("P-Value Right", 0), 10);
		rightMasterTalon.config_kI(0, prefs.getDouble("I-Value Right", 0), 10);
		rightMasterTalon.config_kD(0, prefs.getDouble("D-Value Right", 0), 10);
		rightMasterTalon.config_kF(0, prefs.getDouble("F-Gain Right", 0), 10);
		rightMasterTalon.config_IntegralZone(0, prefs.getInt("I-Zone Right", 0), 10);
		rightMasterTalon.setIntegralAccumulator(0, 0, 10);
		rightMasterTalon.configAllowableClosedloopError(0, prefs.getInt("Tolerable Error Right", 0), 10);
		rightMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
		 
		leftMasterTalon.config_kP(0, prefs.getDouble("P-Value Left", 0), 10);
		leftMasterTalon.config_kI(0, prefs.getDouble("I-Value Left", 0), 10);
		leftMasterTalon.config_kD(0, prefs.getDouble("D-Value Left", 0), 10);
		leftMasterTalon.config_kF(0, prefs.getDouble("F-Gain Left", 0), 10);
		leftMasterTalon.config_IntegralZone(0, prefs.getInt("I-Zone Left", 0), 10);
		leftMasterTalon.setIntegralAccumulator(0, 0, 10);
		leftMasterTalon.configAllowableClosedloopError(0, prefs.getInt("Tolerable Error Left", 0), 10);
		leftMasterTalon.getSensorCollection().setQuadraturePosition(0, 10);
	}
	
	public void noAutoCase() {
		rightMasterTalon.set(ControlMode.PercentOutput, 0);
		leftMasterTalon.set(ControlMode.PercentOutput, 0);
	}
	
	public void setMotionProfile(double[][] rightProf, double[][] leftProf) {
		rightProfile = new MotionProfileExample (rightMasterTalon, rightProf);
		leftProfile = new MotionProfileExample (leftMasterTalon, leftProf);
		
		rightProfile.setIsLast(false);
		leftProfile.setIsLast(false);
		
		rightProfile.reset();
		leftProfile.reset();
		
		rightProfile.startMotionProfile();
		leftProfile.startMotionProfile();
		
		tankDrive.resetGyro();
		start = true;
	}
	
	public void printEncoderValues() {
		double rightEnc = tankDrive.getRightMasterDrive().getSensorCollection().getQuadraturePosition();
		double leftEnc = tankDrive.getLeftMasterDrive().getSensorCollection().getQuadraturePosition();
		
		SmartDashboard.putNumber("Encoder Left Master: ", leftEnc);
		SmartDashboard.putNumber("Encoder Right Master: ", rightEnc);
		SmartDashboard.putNumber("Encoder Left Slave: ", tankDrive.getLeftSlaveDrive().getSensorCollection().getQuadraturePosition());
		SmartDashboard.putNumber("Encoder Right Slave: ", tankDrive.getRightSlaveDrive().getSensorCollection().getQuadraturePosition());
		
		SmartDashboard.putNumber("Left Encoder Distance: ", leftEnc/80/7.3*6*Math.PI);
		SmartDashboard.putNumber("Right Encoder Distance: ", rightEnc/80/7.3*6*Math.PI);
		
		SmartDashboard.putNumber("Gyroscope Value", tankDrive.getAngle());
		
		SmartDashboard.putBoolean("Up State:", elevatorMaxUpToggle);
		SmartDashboard.putBoolean("Down State:", elevatorMaxDownToggle);
		
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
		
		SmartDashboard.putString("BIG FAT CAMERA PLACE HOLDER", " ");
	}
	
	
}