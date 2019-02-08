/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.commands.DriveWithController;
import frc.robot.Robot;
import frc.robot.RobotMap;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem implements PIDOutput {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	private static final VictorSPX leftFront = new VictorSPX(RobotMap.LEFT_FRONT);
	private static final VictorSPX rightFront = new VictorSPX(RobotMap.RIGHT_FRONT);
	private static final WPI_TalonSRX leftBack = new WPI_TalonSRX(RobotMap.LEFT_BACK);
	private static final WPI_TalonSRX rightBack = new WPI_TalonSRX(RobotMap.RIGHT_BACK);

	DifferentialDrive drive = new DifferentialDrive(leftBack, rightBack); 

	AHRS ahrs;

	PIDController turnController;

	public static final double DEAD_ZONE = 0.2;

	// slows down the robot so that the robot is not too fast
	public static final double JOYSTICK_CONSTANT = 1;
	public static final double TRIGGERS_CONSTANT = 1;

	// these slow down the robot for precision driving
	boolean slowMode = false;
	public static final double SLOW_MODE_JOYSTICK = 0.7;
	public static final double SLOW_MODE_TRIGGERS = 0.8;

	double _targetAngle = 0;

	public static final int kEncoderUnitsPerRevolution = 4096;
	public final static int kTimeoutMs = 30;

	    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.025;
    static final double kI = 0.00;
    static final double kD = 0.1;
    static final double kF = 0.00;
    
	static final double kToleranceDegrees = 2.0f; 
	static final double kToleranceSpeed = 200;   

	static double kTargetAngleDegrees;
	
	double rotateToAngleRate;

	@Override
  	public void initDefaultCommand() {
    	// Set the default command for a subsystem here.
    	// setDefaultCommand(new MySpecialCommand());

    	setDefaultCommand(new DriveWithController());
	}

	public DriveTrain() {
		rightBack.configFactoryDefault();
		rightFront.configFactoryDefault();
		leftBack.configFactoryDefault();
		leftFront.configFactoryDefault();

		rightFront.set(ControlMode.PercentOutput, 0);
		rightBack.set(ControlMode.PercentOutput, 0);
		leftFront.set(ControlMode.PercentOutput, 0);
		leftBack.set(ControlMode.PercentOutput, 0);
		
		/* Set Neutral Mode */
		leftBack.setInverted(false);
        rightBack.setInverted(false);
        rightFront.setInverted(true);
		leftFront.setInverted(false);
		
		leftFront.follow(leftBack);
		rightFront.follow(rightBack);

        leftBack.setNeutralMode(NeutralMode.Brake);
        rightBack.setNeutralMode(NeutralMode.Brake);
        rightFront.setNeutralMode(NeutralMode.Brake);
        leftFront.setNeutralMode(NeutralMode.Brake);
	
		leftBack.setSensorPhase(true);
		rightBack.setSensorPhase(false);

		drive.setExpiration(0.1);

		try {
			/* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
			/* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
			ahrs = new AHRS(SPI.Port.kMXP); 
		} catch (RuntimeException ex ) {
			DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
		}

		turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-0.5, 0.5);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.disable();
        
        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
        /* tuning of the Turn Controller's P, I and D coefficients.            */
        /* Typically, only the P value needs to be modified.                   */
        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);
	}

	public void zeroSensors() {
		if(drive.isSafetyEnabled()) drive.setSafetyEnabled(false);
		ahrs.resetDisplacement();
		ahrs.zeroYaw();
		leftBack.getSensorCollection().setQuadraturePosition(0,  kTimeoutMs);
		rightBack.getSensorCollection().setQuadraturePosition(0,  kTimeoutMs);
		System.out.println("[Quadrature Encoders] All sensors are zeroed.\n");		
	}
	
  	/** Checks if the triggers are not being used at all, returns the current to use for driving */
  	public static double deadZone(double current) {
		if (Math.abs(current) < DEAD_ZONE) return 0;
		
    	return (current - DEAD_ZONE * (current > 0d ? 1d : -1d)) / (1d - DEAD_ZONE);
	}
  
	public void turnToAngle(double degrees) {
		drive.setSafetyEnabled(true);
		kTargetAngleDegrees = degrees;
		if (!turnController.isEnabled()) {
			turnController.setSetpoint(degrees);
			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}
		double leftStickValue = rotateToAngleRate;
		double rightStickValue = -rotateToAngleRate;
		drive.tankDrive(leftStickValue,  rightStickValue);
	}

	public void arcadeDrive() {
		drive.setSafetyEnabled(true);

		double leftTrigger = Robot.m_oi.getController(1).getTriggerAxis(Hand.kLeft);
		double rightTrigger = Robot.m_oi.getController(1).getTriggerAxis(Hand.kRight);

		double forward = 1 * Robot.m_oi.getController(1).getY(Hand.kLeft);
		double turn = rightTrigger - leftTrigger;

		forward = deadZone(forward);
		turn = deadZone(turn);

		rightFront.follow(rightBack);
		leftFront.follow(leftBack);

		rightBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
		leftBack.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

		SmartDashboard.putNumber("LeftEncoder: ", leftBack.getSelectedSensorPosition());
		SmartDashboard.putNumber("RightEncoder: ", rightBack.getSelectedSensorPosition());
  	}
  
	public void driveStraight() {
		if(!turnController.isEnabled()) {
			// Acquire current yaw angle, using this as the target angle.
			turnController.setSetpoint(ahrs.getYaw());
			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}
		if(drive.isSafetyEnabled()) drive.setSafetyEnabled(false);
		double magnitude = Robot.m_oi.getController(1).getY(Hand.kLeft);
		double leftStickValue = magnitude + rotateToAngleRate;
		double rightStickValue = magnitude - rotateToAngleRate;
		drive.tankDrive(leftStickValue,  rightStickValue);
	}

	public void disableTurnController() {
		turnController.disable();
	}

	public boolean isTurnToAngleFinished() {
		double angleDifference = Math.abs(ahrs.getYaw() - kTargetAngleDegrees);
		double totalSpeed = Math.abs(leftBack.getSelectedSensorVelocity()) + Math.abs(rightBack.getSelectedSensorVelocity());

		if(angleDifference < kToleranceDegrees && totalSpeed < kToleranceSpeed) return true;
		else return false;
	}
  
  	public void stop() {		
		rightFront.set(ControlMode.PercentOutput, 0);
		rightBack.set(ControlMode.PercentOutput, 0);
		leftFront.set(ControlMode.PercentOutput, 0);
		leftBack.set(ControlMode.PercentOutput, 0);
		
 	}

  	public void setSlowMode(boolean b) {
    	slowMode = b;
	}
  
  	public void switchSlowMode() {
    	slowMode = !slowMode;
  	}
	  
	public void driveStraightFirstCall() {
		System.out.println("This is Drive Straight first call");
	}

	public void arcadeDriveFirstCall() {
		System.out.println("This is Arcade Drive.\n");
		}

	public void tankDriveFirstCall() {
		System.out.println("This is Tank Drive");
	}

	public void printAngle() {
		System.out.println(ahrs.getAngle());
	}

	@Override
	public void pidWrite(double output) {
		rotateToAngleRate = output;
	}
}

