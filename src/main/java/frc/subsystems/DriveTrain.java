/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.commands.DriveWithController;
import frc.commands.SlowMode;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
    public static final int frontLeft = 0;
    public static final int frontRight = 1;
    public static final int backLeft = 2;
    public static final int backRight = 3;

    private static final WPI_TalonSRX talon1 = new WPI_TalonSRX(frontLeft);
    private static final WPI_TalonSRX talon2 = new WPI_TalonSRX(frontRight);
    private static final WPI_TalonSRX talon3 = new WPI_TalonSRX(backLeft);
    private static final WPI_TalonSRX talon4 = new WPI_TalonSRX(backRight);

    public static SpeedControllerGroup left = new SpeedControllerGroup(talon1, talon3);
    public static SpeedControllerGroup right = new SpeedControllerGroup(talon2, talon4);

   public DifferentialDrive drive = new DifferentialDrive(left, right);
   public static final double deadZoneConstant = 0.2;
   
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveWithController());
  }
  public double slowModeConstant = 0.5;
  private boolean slowMode = false;
  public void setSlowMode(boolean b) {
    slowMode = b;
  }
  // implement this to drive with a controller
  public void drive() {
    double leftY = Robot.m_oi.getController().getY(Hand.kLeft);
    double leftTrigger = Robot.m_oi.getController().getTriggerAxis(Hand.kLeft);
    double rightTrigger = Robot.m_oi.getController().getTriggerAxis(Hand.kRight);
    double turn;
    Robot.kDriveTrain.drive();
    if (leftY < deadZoneConstant) {
      leftY = 0;
    }
    if (leftTrigger < deadZoneConstant) {
      leftTrigger = 0;
    }
    if (rightTrigger < deadZoneConstant) {
      rightTrigger = 0;
    }
    turn = leftTrigger - rightTrigger;
    if (slowMode = true) {
      leftY *= slowModeConstant;
    }
    drive.arcadeDrive(leftY, turn);
  }
  public void stop() {

  }
}
