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
import frc.commands.DriveWithController;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;


/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static final int LEFT_FRONT = 0;
  public static final int RIGHT_FRONT = 1;
  public static final int LEFT_BACK = 2;
  public static final int RIGHT_BACK = 3;

  private static final WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT);
  private static final WPI_TalonSRX rightFront = new WPI_TalonSRX(RIGHT_FRONT);
  private static final WPI_TalonSRX leftBack = new WPI_TalonSRX(LEFT_BACK);
  private static final WPI_TalonSRX rightBack = new WPI_TalonSRX(RIGHT_BACK);

  public static final SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
  public static final SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);

  public static final double DEAD_ZONE = 0.2;

  // slows down the robot so that the robot is not too fast
  public static final double JOYSTICK_CONSTANT = 0.7;
  public static final double TRIGGERS_CONSTANT = 0.7;

  // these slow down the robot for precision driving
  static boolean slowMode = false;
  public static final double SLOW_MODE_JOYSTICK = 0.7;
  public static final double SLOW_MODE_TRIGGERS = 0.8;

  boolean _firstCall = false;
  boolean _state = false;
  double _targetAngle = 0;

  private static final DifferentialDrive drive = new DifferentialDrive(left, right);

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveWithController());
  }

  public DriveTrain() {
	rightFront.set(ControlMode.PercentOutput, 0);
	leftFront.set(ControlMode.PercentOutput, 0);
	
	/* Set Neutral Mode */
	leftFront.setNeutralMode(NeutralMode.Brake);
	rightFront.setNeutralMode(NeutralMode.Brake);
	
	/** Closed loop configuration */
	
	/* Configure the drivetrain's left side Feedback Sensor as a Quadrature Encoder */
	leftFront.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder,			// Local Feedback Source
												Constants.PID_PRIMARY,				// PID Slot for Source [0, 1]
												Constants.kTimeoutMs);				// Configuration Timeout

	/* Configure the left Talon's Selected Sensor to be a remote sensor for the right Talon */
	rightFront.configRemoteFeedbackFilter(leftFront.getDeviceID(),					// Device ID of Source
											RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
											Constants.REMOTE_0,							// Source number [0, 1]
											Constants.kTimeoutMs);						// Configuration Timeout
	
	/* Setup difference signal to be used for turn when performing Drive Straight with encoders */
	rightFront.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);	// Feedback Device of Remote Talon
	rightFront.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, Constants.kTimeoutMs);		// Quadrature Encoder of current Talon
	
	/* Difference term calculated by right Talon configured to be selected sensor of turn PID */
	rightFront.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
												Constants.PID_TURN, 
												Constants.kTimeoutMs);
	
	/* Scale the Feedback Sensor using a coefficient */
	/**
	 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
	 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
	 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
	 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
	 *  and motion profile trajectory points can range +-2 rotations.
	 */
	rightFront.configSelectedFeedbackCoefficient(	Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation,	// Coefficient
													Constants.PID_TURN, 														// PID Slot of Source
													Constants.kTimeoutMs);														// Configuration Timeout
	
	/* Configure output and sensor direction */
	leftFront.setInverted(false);
	leftFront.setSensorPhase(true);
	rightFront.setInverted(true);
	rightFront.setSensorPhase(true);
	
	/* Set status frame periods */
	rightFront.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
	rightFront.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);
	leftFront.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);		//Used remotely by right Talon, speed up

	/* Configure neutral deadband */
	rightFront.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
	leftFront.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

	/* max out the peak output (for all modes).  However you can
	 * limit the output of a given PID object with configClosedLoopPeakOutput().
	 */
	leftFront.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
	leftFront.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
	rightFront.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
	rightFront.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);

	/* FPID Gains for turn servo */
	rightFront.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
	rightFront.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
	rightFront.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
	rightFront.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
	rightFront.config_IntegralZone(Constants.kSlot_Turning, Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
	rightFront.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
	rightFront.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);
		
	/* 1ms per loop.  PID loop can be slowed down if need be.
	 * For example,
	 * - if sensor updates are too slow
	 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
	 * - sensor movement is very slow causing the derivative error to be near zero.
	 */
	int closedLoopTimeMs = 1;
	rightFront.configClosedLoopPeriod(0, closedLoopTimeMs, Constants.kTimeoutMs);
	rightFront.configClosedLoopPeriod(1, closedLoopTimeMs, Constants.kTimeoutMs);

	/* configAuxPIDPolarity(boolean invert, int timeoutMs)
	 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
	 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
	 */
	rightFront.configAuxPIDPolarity(false, Constants.kTimeoutMs);

	/* Initialize */
	_firstCall = true;
	_state = false;
	zeroSensors();
  }

  void zeroSensors() {
	leftFront.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
	rightFront.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
	System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");
}
  /** Checks if the triggers are not being used at all, returns the current to use for driving */
  public static double deadZone(double current) {
    if (Math.abs(current) < DEAD_ZONE) return 0;
    
    return (current - DEAD_ZONE * (current > 0d ? 1d : -1d)) / (1d - DEAD_ZONE);
  }

  // implement this to drive with a controller
  public void tankDrive() {
    double leftY = -Robot.m_oi.getController().getY(Hand.kLeft);
    double rightY = -Robot.m_oi.getController().getY(Hand.kRight);
    
    leftY = deadZone(leftY);
    rightY = deadZone(leftY);

    leftY = leftY * (slowMode ? SLOW_MODE_JOYSTICK : 1d) * JOYSTICK_CONSTANT;
    rightY = rightY * (slowMode ? SLOW_MODE_JOYSTICK : 1d) * JOYSTICK_CONSTANT;

    drive.tankDrive(leftY, rightY);
  }
  
  public void arcadeDrive() {
    double leftY = -Robot.m_oi.getController().getY(Hand.kLeft);
    double leftTrigger = Robot.m_oi.getController().getTriggerAxis(Hand.kLeft);
    double rightTrigger = Robot.m_oi.getController().getTriggerAxis(Hand.kRight);

    leftY = deadZone(leftY);

	leftY = leftY * (slowMode ? SLOW_MODE_JOYSTICK : 1d) * JOYSTICK_CONSTANT;
	
    double rotation = (rightTrigger - leftTrigger) *
  (slowMode ? SLOW_MODE_TRIGGERS : 1d) * TRIGGERS_CONSTANT;

    drive.arcadeDrive(leftY, rotation);
  }

  public void driveStraight() {
    double forward = -1 * Robot.m_oi.getController().getY();
		double turn = (Robot.m_oi.getController().getTriggerAxis(Hand.kRight) - 
		Robot.m_oi.getController().getTriggerAxis(Hand.kLeft)) *
		 (slowMode ? SLOW_MODE_TRIGGERS : 1d) * TRIGGERS_CONSTANT;
		forward = deadZone(forward);
						
		if(!_state){
			if (_firstCall)
				System.out.println("This is Arcade Drive.\n");
			
			leftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			rightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
		}else{
			if (_firstCall) {
				System.out.println("This is Drive Straight using the auxiliary feature with" + 
					"the difference between two encoders to maintain current heading.\n");
				
				/* Determine which slot affects which PID */
				rightFront.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}
			
			/* Configured for percentOutput with Auxiliary PID on Quadrature Encoders' Difference */
			rightFront.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, _targetAngle);
			rightBack.follow(rightFront, FollowerType.AuxOutput1);
			leftFront.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, _targetAngle);
			leftBack.follow(rightFront, FollowerType.AuxOutput1);
		}
		_firstCall = false;
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
}

