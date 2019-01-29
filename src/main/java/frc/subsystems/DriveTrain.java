/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.commands.DriveWithController;
import frc.robot.Gains;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private static final WPI_TalonSRX leftFront = new WPI_TalonSRX(RobotMap.LEFT_FRONT);
	private static final WPI_TalonSRX rightFront = new WPI_TalonSRX(RobotMap.RIGHT_FRONT);
	private static final WPI_TalonSRX leftBack = new WPI_TalonSRX(RobotMap.LEFT_BACK);
	private static final WPI_TalonSRX rightBack = new WPI_TalonSRX(RobotMap.RIGHT_BACK);

	public static final SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
	public static final SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);

	public static final double DEAD_ZONE = 0.2;

	// slows down the robot so that the robot is not too fast
	public static final double JOYSTICK_CONSTANT = 1;
	public static final double TRIGGERS_CONSTANT = 1;

	// these slow down the robot for precision driving
	boolean slowMode = false;
	public static final double SLOW_MODE_JOYSTICK = 0.7;
	public static final double SLOW_MODE_TRIGGERS = 0.8;

	static final double leftkF = 1.02,
			leftkP = 1.2,
			leftkI = 0.004,
			leftkD = 8.0,
			rightkF = 1.02,
			rightkP = 1.2,
			rightkI = 0.004,
			rightkD = 8.0;

	double _targetAngle = 0;
	DriveType curDriveType;

	AHRS ahrs;

	public enum DriveType {
		kArcade, kDriveStraight, kTank
	}

	/**
	 * Using the configSelectedFeedbackCoefficient() function, scale units to 3600 per rotation.
	 * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
	 */
	public final static double kTurnTravelUnitsPerRotation = 3600;

	/**
	 * Empirically measure what the difference between encoders per 360'
	 * Drive the robot in clockwise rotations and measure the units per rotation.
	 * Drive the robot in counter clockwise rotations and measure the units per rotation.
	 * Take the average of the two.
	 */
	public final static int kEncoderUnitsPerRotation = 51711;

	/**
	 * Set to zero to skip waiting for confirmation.
	 * Set to nonzero to wait and report to DS if action fails.
	 */
	public final static int kTimeoutMs = 30;

	/**
	 * Motor neutral dead-band, set to the minimum 0.1%.
	 */
	public final static double kNeutralDeadband = 0.001;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
	 * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity units at 100% output
	 * Not all set of Gains are used in this project and may be removed as desired.
	 * 
	 * 	                                    			  kP   kI   kD   kF               Iz    PeakOut */
	public final static Gains kGains_Distanc = new Gains( 0.1, 0.0,  0.0, 0.0,            100,  0.50 );
	public final static Gains kGains_Turning = new Gains( 3.0, 0.0,  4.0, 0.0,            200,  1.00 );
	public final static Gains kGains_Velocit = new Gains( 0.1, 0.0, 20.0, 1023.0/6800.0,  300,  0.50 );
	public final static Gains kGains_MotProf = new Gains( 1.0, 0.0,  0.0, 1023.0/6800.0,  400,  1.00 );

	/** ---- Flat constants, you should not need to change these ---- */
	/* We allow either a 0 or 1 when selecting an ordinal for remote devices [You can have up to 2 devices assigned remotely to a talon/victor] */
	public final static int REMOTE_0 = 0;
	public final static int REMOTE_1 = 1;
	/* We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1 is auxiliary */
	public final static int PID_PRIMARY = 0;
	public final static int PID_TURN = 1;
	/* Firmware currently supports slots [0, 3] and can be used for either PID Set */
	public final static int SLOT_0 = 0;
	public final static int SLOT_1 = 1;
	public final static int SLOT_2 = 2;
	public final static int SLOT_3 = 3;
	/* ---- Named slots, used to clarify code ---- */
	public final static int kSlot_Distanc = SLOT_0;
	public final static int kSlot_Turning = SLOT_1;
	public final static int kSlot_Velocit = SLOT_2;
	public final static int kSlot_MotProf = SLOT_3;

	private static final DifferentialDrive drive = new DifferentialDrive(left, right);
  
	@Override
  	public void initDefaultCommand() {
    	// Set the default command for a subsystem here.
    	// setDefaultCommand(new MySpecialCommand());

    	setDefaultCommand(new DriveWithController());
	}

	public DriveTrain() {
		SmartDashboard.putNumber("kP", 3.0);
		SmartDashboard.putNumber("kI", 0.0);
		SmartDashboard.putNumber("kD", 4.0);
		SmartDashboard.putNumber("kF", 0.0);

		curDriveType = DriveType.kArcade;

		rightFront.set(ControlMode.PercentOutput, 0);
		leftFront.set(ControlMode.PercentOutput, 0);
		
		/* Set Neutral Mode */
		leftFront.setNeutralMode(NeutralMode.Brake);
		rightFront.setNeutralMode(NeutralMode.Brake);
		leftBack.setNeutralMode(NeutralMode.Brake);
		rightBack.setNeutralMode(NeutralMode.Brake);
		
		/** Closed loop configuration */
		
		/* Configure the drivetrain's left side Feedback Sensor as a Quadrature Encoder */
		leftFront.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,	PID_PRIMARY, kTimeoutMs);
		/* Configure the left Talon's Selected Sensor to be a remote sensor for the right Talon */
		rightFront.configRemoteFeedbackFilter(leftFront.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, REMOTE_0, kTimeoutMs);	
		/* Setup difference signal to be used for turn when performing Drive Straight with encoders */
		rightFront.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, kTimeoutMs);
		rightFront.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, kTimeoutMs);
		
		/* Difference term calculated by right Talon configured to be selected sensor of turn PID */
		rightFront.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, PID_TURN, kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		rightFront.configSelectedFeedbackCoefficient(kTurnTravelUnitsPerRotation / kEncoderUnitsPerRotation, PID_TURN, kTimeoutMs);														// Configuration Timeout
		
		/* Configure output and sensor direction */
		leftBack.setInverted(false);
		leftBack.setSensorPhase(true);
		rightBack.setInverted(true);
		rightBack.setSensorPhase(true);
		rightFront.setInverted(false);
		leftFront.setInverted(false);
		
		/* Set status frame periods */
		rightFront.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, kTimeoutMs);
		rightFront.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, kTimeoutMs);
		leftFront.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,  kTimeoutMs);		//Used remotely by right Talon, speed up

		/* Configure neutral deadband */
		rightFront.configNeutralDeadband( kNeutralDeadband,  kTimeoutMs);
		leftFront.configNeutralDeadband( kNeutralDeadband,  kTimeoutMs);

		/* max out the peak output (for all modes).  However you can
		* limit the output of a given PID object with configClosedLoopPeakOutput().
		*/
		leftFront.configPeakOutputForward(+1.0,  kTimeoutMs);
		leftFront.configPeakOutputReverse(-1.0,  kTimeoutMs);
		rightFront.configPeakOutputForward(+1.0,  kTimeoutMs);
		rightFront.configPeakOutputReverse(-1.0,  kTimeoutMs);

		/* FPID Gains for turn servo */
		rightFront.config_kP( kSlot_Turning,  kGains_Turning.kP,  kTimeoutMs);
		rightFront.config_kI( kSlot_Turning,  kGains_Turning.kI,  kTimeoutMs);
		rightFront.config_kD( kSlot_Turning,  kGains_Turning.kD,  kTimeoutMs);
		rightFront.config_kF( kSlot_Turning,  kGains_Turning.kF,  kTimeoutMs);
		rightFront.config_IntegralZone( kSlot_Turning,  kGains_Turning.kIzone,  kTimeoutMs);
		rightFront.configClosedLoopPeakOutput( kSlot_Turning,  kGains_Turning.kPeakOutput,  kTimeoutMs);
		rightFront.configAllowableClosedloopError( kSlot_Turning, 0,  kTimeoutMs);
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		* For example,
		* - if sensor updates are too slow
		* - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		* - sensor movement is very slow causing the derivative error to be near zero.
		*/
		int closedLoopTimeMs = 1;
		rightFront.configClosedLoopPeriod(0, closedLoopTimeMs,  kTimeoutMs);
		rightFront.configClosedLoopPeriod(1, closedLoopTimeMs,  kTimeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		* false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		* true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		*/
		rightFront.configAuxPIDPolarity(false,  kTimeoutMs);
		zeroSensors();

		try {
			/* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
			/* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
			ahrs = new AHRS(SPI.Port.kMXP); 
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}
	}

	public void zeroSensors() {
		leftFront.getSensorCollection().setQuadraturePosition(0,  kTimeoutMs);
		rightFront.getSensorCollection().setQuadraturePosition(0,  kTimeoutMs);
		System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
	}
	
  	/** Checks if the triggers are not being used at all, returns the current to use for driving */
  	public static double deadZone(double current) {
		if (Math.abs(current) < DEAD_ZONE) return 0;
		
    	return (current - DEAD_ZONE * (current > 0d ? 1d : -1d)) / (1d - DEAD_ZONE);
	}

	public void drive() {
		switch(curDriveType) {
			default:
			case kArcade:
				arcadeDrive();
				break;
			case kTank:
				tankDrive();
				break;
			case kDriveStraight:
				driveStraight();
				break;
		}
	}

  	// implement this to drive with a controller
  	public void tankDrive() {
		double leftY = -Robot.m_oi.getController().getY(Hand.kLeft);
		double rightY = -Robot.m_oi.getController().getY(Hand.kRight);
		
		leftY = deadZone(leftY);
		rightY = deadZone(rightY);

		leftY *= (slowMode ? SLOW_MODE_JOYSTICK : 1d) * JOYSTICK_CONSTANT;
		rightY *= (slowMode ? SLOW_MODE_JOYSTICK : 1d) * JOYSTICK_CONSTANT;

		drive.tankDrive(leftY, rightY);
  	}
  
	public void arcadeDrive() {
		double leftY = -Robot.m_oi.getController().getY(Hand.kLeft);
		double leftTrigger = Robot.m_oi.getController().getTriggerAxis(Hand.kLeft);
		double rightTrigger = Robot.m_oi.getController().getTriggerAxis(Hand.kRight);

		leftY = deadZone(leftY);
		System.out.println(leftY);

		leftY *= (slowMode ? SLOW_MODE_JOYSTICK : 1d) * JOYSTICK_CONSTANT;
		
		double rotation = (rightTrigger - leftTrigger) *
			(slowMode ? SLOW_MODE_TRIGGERS : 1d) * TRIGGERS_CONSTANT;

		drive.arcadeDrive(leftY, rotation);
  	}
  
	public void driveStraight() {
		double forward = -1 * Robot.m_oi.getController().getY(Hand.kLeft);
		forward = deadZone(forward) * (slowMode ? SLOW_MODE_JOYSTICK : 1d) * JOYSTICK_CONSTANT;
		System.out.println("This is Drive Straight using the auxiliary feature with" + 
			"the difference between two encoders to maintain current heading.\n");

		/* Configured for percentOutput with Auxiliary PID on Quadrature Encoders' Difference */
		rightBack.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, _targetAngle);
		leftBack.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, _targetAngle);	
	}
  
  	public void stop() {
    	drive.tankDrive(0, 0);
 	}

  	public void setSlowMode(boolean b) {
    	slowMode = b;
	}
  
  	public void switchSlowMode() {
    	slowMode = !slowMode;
  	}
	  
	public void driveStraightFirstCall() {
		System.out.println("This is Drive Straight using the auxiliary feature with" + 
					"the difference between two encoders to maintain current heading.\n");

		/* Determine which slot affects which PID */
		rightBack.selectProfileSlot( kSlot_Turning,  PID_TURN);
	}

	public void arcadeDriveFirstCall() {
		System.out.println("This is Arcade Drive.\n");
	}

	public void printAngle() {
		System.out.println(ahrs.getAngle());
	}

	public void switchDriveType(DriveType d) {
		curDriveType = d;
		switch(curDriveType) {
			case kArcade:
				arcadeDriveFirstCall();
				break;
			case kDriveStraight:
				driveStraightFirstCall();
				break;
			default:
				break;
		}
	}
}

