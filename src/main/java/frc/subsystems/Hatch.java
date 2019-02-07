/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

	public static final double ARM_SPEED = 0.1;
	public static final double EXTENSION_SPEED = 0.1;
	public static final double INTERVAL = 1d;

  static final WPI_TalonSRX armMotor = new WPI_TalonSRX(RobotMap.ARMMOTOR);
	static final Spark extensionMotor = new Spark(RobotMap.EXTENSION);
	
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

	public void extend() {
		extensionMotor.set(EXTENSION_SPEED);
	}
}
