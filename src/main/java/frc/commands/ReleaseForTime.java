/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ReleaseForTime extends Command {

  double time;

  public ReleaseForTime(double seconds) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.kHatch);

    time = seconds;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    try {
      setTimeout(time);
    } catch (IllegalArgumentException e) {
      System.out.println(e.toString());
      setTimeout(0);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.kHatch.release();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.kHatch.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
