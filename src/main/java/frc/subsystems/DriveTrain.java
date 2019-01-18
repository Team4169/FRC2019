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

  private static final DifferentialDrive drive = new DifferentialDrive(left, right);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveWithController());
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

  public void stop() {
    drive.tankDrive(0, 0);
  }

  public void setSlowMode(boolean b) {
    slowMode = b;
  }
}
