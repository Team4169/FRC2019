/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hatch extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	// 20:1 gearbox, 12 counts per revolution, 70 degrees

	public static final double ARM_SPEED = 0.05;
	public static final double EXTENSION_SPEED = 0.3;
	public static final double INTERVAL = 1d;
	public static final double GEARBOX_RATIO = 20d/1d;
	public static final int COUNTS_PER_REVOLUTION = 12;
	public static final double TURNING_ANGLE = 70d;
	public static final int ENCODER_THRESHOLD = 7;

	public static final int MAX_COUNTS = (int) Math.floor(GEARBOX_RATIO * ((double) COUNTS_PER_REVOLUTION) * TURNING_ANGLE / 360d);
	/* measure this angle and make sure it is less than the actual angle */
	public static final int APPROX_START_POS = MAX_COUNTS / 2;

	int kTimeoutMs = 30;

	final WPI_TalonSRX armMotor;
	final Spark extensionMotor;

	boolean normalSwitchMode;
	DigitalInput extensionLimitSwitch;

	public Hatch() {
		extensionLimitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH_PORT);
		normalSwitchMode = extensionLimitSwitch.get();
		armMotor = new WPI_TalonSRX(RobotMap.ARMMOTOR);
		extensionMotor = new Spark(RobotMap.EXTENSION);

		armMotor.configFactoryDefault();

		armMotor.setInverted(true);
		armMotor.set(ControlMode.PercentOutput, 0.0);
		armMotor.setNeutralMode(NeutralMode.Brake);

		armMotor.setSensorPhase(false);

		// This assumes the robot starts with the hatch staged
		armMotor.setSelectedSensorPosition(APPROX_START_POS);

	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}

	public void grab() {
		armMotor.set(ARM_SPEED);
	}

	public void release() {
		armMotor.set(-ARM_SPEED);
	}

	public void stop() {
		armMotor.set(0);
	}

	public void extend() {
		extensionMotor.set(EXTENSION_SPEED);
	}
	/** */
	public boolean isGrabbed() {
		return armMotor.getSelectedSensorPosition() <= ENCODER_THRESHOLD;
	}

	public boolean isReleased() {
		return armMotor.getSelectedSensorPosition() >= MAX_COUNTS - ENCODER_THRESHOLD;
	}

	public boolean getLimitSwitch() {
		return extensionLimitSwitch.get() != normalSwitchMode;
	}

	public void zeroSensors() {
		armMotor.setSelectedSensorPosition(0);
	}

	public void stopExtend() {
		extensionMotor.stopMotor();
	}

	public int getEncoderValue() {
		return armMotor.getSelectedSensorPosition();
	}
}
