/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

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

	public static final double ARM_SPEED = 0.1;
	public static final double EXTENSION_SPEED = 0.1;
	public static final double INTERVAL = 1d;
	public static final double GEARBOX_RATIO = 20d/1d;
	public static final int COUNTS_PER_REVOLUTION = 12;
	public static final int TURNING_ANGLE = 70;
	public static final int ENCODER_THRESHOLD = 20;

  static final WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.ARMMOTOR);
	static final Spark extensionMotor = new Spark(RobotMap.EXTENSION);

	boolean normalSwitchMode;
	DigitalInput extensionLimitSwitch;

	public Hatch() {
		extensionLimitSwitch = new DigitalInput(RobotMap.LIMIT_SWITCH_PORT);
		normalSwitchMode = extensionLimitSwitch.get();
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
		return armMotor.getSelectedSensorPosition() >=
				GEARBOX_RATIO * COUNTS_PER_REVOLUTION * TURNING_ANGLE / 360d - ENCODER_THRESHOLD;
	}

	public boolean getLimitSwitch() {
		return extensionLimitSwitch.get() != normalSwitchMode;
	}
}
