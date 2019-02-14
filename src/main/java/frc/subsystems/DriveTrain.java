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

	// speed at which the robot turns during findTarget
	double TURN_CONSTANT = 0.3;

	double _targetAngle = 0;

	public static final int kEncoderUnitsPerRevolution = 4096;
	public final static int kTimeoutMs = 30;

	/* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
	/* and D constants and test the mechanism.                         */
	
	//static double PTV_ERROR = -1d;
    
    static final double kP = 0.025;
    static final double kI = 0.00;
    static final double kD = 0.1;
    static final double kF = 0.00;
    
	static final double kToleranceDegrees = 2.0f; 
	static final double kToleranceSpeed = 200;   

	static double kTargetAngleDegrees;
	
	double rotateToAngleRate;

	// This is the RobotModel
	// Well-known constants
	// Max motor voltage (= 1.0 motor power)
	public static final double maxVoltage = 12.0;           // volts
	public static final double maxMotorSpeed = (5300.0/60); // rev / sec, for CIM motor
	public static final double wheelDiam = 6.0;             // in, for kitbot chassis
	public static final double gearReduction = 10.7;        // for kitbot chassis
	public static final double maxVelocity = (Math.PI*wheelDiam*maxMotorSpeed) / gearReduction;
	public static final double secPerStep = 0.02;          // time per "step" (sched run)

	// Compute kV; can replace with empirical
	// value if needed
	public static final double kV = maxVoltage / maxVelocity;

	// Can replace startVoltage with empirical
	// value if needed
	public static final double startVoltage = 1.25;         // volts

	/* Time to accelerate from minimum velocity (startVoltage) to maximum
	# velocity (maxVoltage).  This can be tweaked some, but if too small
	# it won't be practical for the robot.  In practice 2 sec seems right. */
	public static final double accelTime = 2.0;             // sec

	// Number of steps required to accelerate from 0 to max velocity
	public static final double accelSteps = accelTime / secPerStep;

	/* Compute velocity per step -- approximation to acceleration
	We want to accelerate from the startVoltage to the maxVoltage
	in accelTime seconds
	Velocity increase (or decrease) for a single step while
	accelerating (or decelerating). */ 
	public static double velocityPerStep;

	/* Power increase per step -- equal to velocity per step
	# converted to power (Note this is a delta, not absolute) */
	public static double powerPerStep;

	// Motor power required to start the robot i.e. overcome friction
	public static double startPower;


	public static double curDistance = 0.0;
	public static double curPower = 0.0;
	public static double curTime = 0.0;      // shouldn't really be here but convenient

	@Override
  	public void initDefaultCommand() {
    	// Set the default command for a subsystem here.
    	// setDefaultCommand(new MySpecialCommand());

    	setDefaultCommand(new DriveWithController());
	}

	public DriveTrain() {
		try {
			velocityPerStep = voltageToVelocity(maxVoltage) / accelSteps;
		} catch(Exception e) {
			System.out.println("Voltage to velocity failed");
		}

		try {
			powerPerStep = voltageToPower(kV * velocityPerStep);
		} catch(Exception e) {
			System.out.println("Voltage to power failed");
		}

		try {
			voltageToPower(startVoltage);
		} catch(Exception e) {
			System.out.println("Voltage to power failed");
		}

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
		System.out.println("[Quadrature Encoders] and AHRS sensors are zeroed.\n");		
	}

	public void zeroGyro() {
		ahrs.zeroYaw();
	}

	public void zeroAccelerometer() {
		ahrs.resetDisplacement();
	}
	
	public void zeroDriveEncoders() {
		leftBack.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
		rightBack.getSensorCollection().setQuadraturePosition(0, kTimeoutMs);
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
	
	public void findTarget() {;
		rightFront.follow(rightBack);
		leftFront.follow(leftBack);

		rightBack.set(ControlMode.PercentOutput, TURN_CONSTANT);
		leftBack.set(ControlMode.PercentOutput, -TURN_CONSTANT);
	}

	public void driveStraight() {
		if(!turnController.isEnabled()) {
			// Acquire current yaw angle, using this as the target angle.
			turnController.setSetpoint(ahrs.getYaw());
			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
			turnController.enable();
		}
		if(drive.isSafetyEnabled()) drive.setSafetyEnabled(false);
		double magnitude = -Robot.m_oi.getController(1).getY(Hand.kLeft);
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
	 
	 public double powerToVoltage(double power) throws Exception {
		if(power >= 0.0 && power <= 1.0) {
			return maxVoltage * power;
		}
		throw(new Exception());
	 }

	 public double voltageToPower(double voltage) throws Exception {
		 if(voltage >= 0.0 && voltage <= maxVoltage) {
			 return voltage / maxVoltage;
		 }
		 throw(new Exception());
	 }

	 public double voltageToVelocity(double voltage) throws Exception {
		 if(voltage >= 0.0 && voltage <= maxVoltage) {
			 if(voltage < startVoltage) {
				 return 0.0;
			 } else {
				return (voltage - startVoltage) / kV;
			 }
		 }
		 
		 throw(new Exception());
	 }

	 /* Note: this only applies for an absolute velocity
	 To compute a voltage delta for a velocity delta,
	 it's just kV * velocity */
	public double velocityToVoltage(double velocity) {
		return (kV * velocity) + startVoltage;
	}

	public double powerToVelocity(double power) throws Exception {
		return voltageToVelocity(powerToVoltage(power));
	}

	public double velocityToPower(double velocity) throws Exception {
		return voltageToPower(velocityToVoltage(velocity));
	}

	/* Calculate the distance travelled while accelerating stepwise
	It is SUM[i=0 to n](i * delta-v * delta-t)
	But as long as delta-v and delta-t are constants they can be
    pulled outside the summation; SUM[i=0 to n](i) = (n)(n-1)/2 */

	public double calculateAccelDistance(double n, double deltaV, double deltaT) {
		return (deltaV * deltaT * n * (n -1)) / 2.0;
	}

	/*Calculate the number of acceleration steps to reach
	(approximately) the specified distance.  This is the
	rough inverse of calculateAccelDistance() above and
	requires finding the positive root of the quadratic. */
	public double calculateAccelSteps(double dist, double deltaV, double deltaT) {
    	double c = -(2.0 * dist) / (deltaV * deltaT);
    	double root = (1.0 + Math.sqrt(1.0 - (4.0*c))) / 2.0;
		return Math.floor(root);
	}
	// This is the end of the RobotModel

	public void driveStraightStep(double power) {
		/*
		Drive straight ahead for one 'step'
		power: Motor power, from 0.0 to 1.0
		*/
		curPower = power;
		double vel;
		try {
			vel = powerToVelocity(power);
		} catch(Exception e) {
			vel = 0.0;
			System.out.println("Power to velocity failed");
		}
		double dist = vel * secPerStep;
		curDistance = curDistance + dist;
		curTime = curTime + secPerStep;		
	}

	public static void resetCurrentDistance() {
		curDistance = 0.0;
	}

	public static double getCurrentDistance() {
		return curDistance;
	}

	public static double getCurrentPower() {
		return curPower;
	}

	public static double getCurrentTime(){
		return curTime;
	}

	

  	public void setSlowMode(boolean b) {
    	slowMode = b;
	}
  
  	public void switchSlowMode() {
    	slowMode = !slowMode;
  	}
	  
	public void driveStraightFirstCall() {
		System.out.println("This is Drive Straight");
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

