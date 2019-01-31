/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.subsystems.DriveTrain;

public class SwitchDriveType extends Command {

  DriveTrain.DriveType type;
  public SwitchDriveType(DriveTrain.DriveType d) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    type = d;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.kDriveTrain.switchDriveType(type);
    System.out.println("Executing switchDriveType");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true; // always is finished (never runs)
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("ending switch drive type");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
