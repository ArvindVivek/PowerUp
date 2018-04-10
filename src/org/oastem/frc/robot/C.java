package org.oastem.frc.robot;

public class C {
	public class Port{
		public static final int RIGHT_MASTER_DRIVE = 3;
		public static final int LEFT_MASTER_DRIVE = 1;
		public static final int RIGHT_SLAVE_DRIVE = 2;
		public static final int LEFT_SLAVE_DRIVE = 0;
		
		public static final int ELEVATOR_PORT = 4; 
		public static final int ELEVATOR_2_PORT = 5; 
		
		public static final int INTAKE_PORT = 0; 
		public static final int INTAKE_2_PORT = 1; 
		
		public static final int WRIST_PORT = 2; 
		
		public static final int ELEVATOR_MIN_PORT = 2;
		public static final int ELEVATOR_MAX_PORT = 5;
		
		public static final int ARTICULATION_MIN_PORT = 1;
		
		public static final int ELEVATOR_ENC_A = 3;
		public static final int ELEVATOR_ENC_B = 4;
	}
	
	public class Distances{
		public static final int SCALE_DIST = 7400;
		public static final int SWITCH_DIST = 5000;
		public static final int IN_BETWEEN_DIST = 6400;
		public static final int TURN_DIST = 590;
		public static final int AFTER_TURN_DIST = 2000;
		
		public static final int MAX_ENC_COUNT = 1000; 
	}
}
