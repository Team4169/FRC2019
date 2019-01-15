/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithController;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new DriveWithController());
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  static WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(RobotMap.leftFrontMotor);
	static WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(RobotMap.leftBackMotor);
	//static WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(RobotMap.rightFrontMotor);
	//static WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(RobotMap.rightBackMotor);
	
	//static SpeedControllerGroup left = new SpeedControllerGroup(leftFrontMotor, leftBackMotor);
	//static SpeedControllerGroup right = new SpeedControllerGroup(rightFrontMotor, rightBackMotor);

  //static DifferentialDrive drive = new DifferentialDrive(left, right);

  static final DifferentialDrive drive = new DifferentialDrive(leftFrontMotor, leftBackMotor);

  public void drive() {

    System.out.println("steven is bad");
    // double speedLeft = Robot.m_oi.getController().getY(GenericHID.Hand.kLeft);
		// double speedRight = Robot.m_oi.getController().getY(GenericHID.Hand.kRight);

    drive.tankDrive(0.1, 0.1);
  }

  public void stop() {
    drive.tankDrive(0, 0);
  }
}
