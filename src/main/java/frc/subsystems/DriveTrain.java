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
import frc.robot.RobotConfiguration;

/**
 * Drives the robot
 */
public class DriveTrain extends Subsystem {
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	
	public static final int LEFT_FRONT = 4;
	public static final int RIGHT_FRONT = 2;
	public static final int LEFT_BACK = 1;
	public static final int RIGHT_BACK = 5;

	private static final WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT);
	private static final WPI_TalonSRX rightFront = new WPI_TalonSRX(RIGHT_FRONT);
	private static final WPI_TalonSRX leftBack = new WPI_TalonSRX(LEFT_BACK);
	private static final WPI_TalonSRX rightBack = new WPI_TalonSRX(RIGHT_BACK);

	public static final SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
	public static final SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);

	//sets the drive motors for arcadeDrive and tankDrive
	private static final DifferentialDrive drive = new DifferentialDrive(left, right);

	private boolean slowMode = false;
	private static final double maxSpeed = 0.5;

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
	}


	//leftY controls forward and backward. rightX controls turn
	public void arcadeDrive() { 
		double leftY = RobotConfiguration.controller1.getY(Hand.kLeft);
		double rightX = RobotConfiguration.controller1.getX(Hand.kRight);

		if(slowMode) {
			leftY = Math.min(leftY, maxSpeed);
			rightX = Math.min(rightX, maxSpeed);
		}
		else {
			leftY = Math.pow(leftY, 3); //cubic to allow finer tuning of lower powers
			rightX = Math.pow(rightX, 3);
		}

		drive.arcadeDrive(leftY, rightX, false);
	}

	//leftY controls left power, rightY controls right power
	public void tankDrive() {
		double leftY = RobotConfiguration.controller1.getY(Hand.kLeft);
		double rightY = RobotConfiguration.controller1.getY(Hand.kRight);

		if(slowMode) {
			leftY = Math.min(leftY, maxSpeed);
			rightY = Math.min(rightY, maxSpeed);
		}
		else {
			leftY = Math.pow(leftY, 3); //cubic to allow finer tuning of lower powers
			rightY = Math.pow(rightY, 3);
		}

		drive.tankDrive(leftY, rightY);

		//For debugging
		//System.out.println(leftFront.getMotorOutputPercent() + ", " + rightFront.getMotorOutputPercent());
	}

	public void stop() {
		drive.tankDrive(0, 0);
		drive.arcadeDrive(0, 0);
	}

}

