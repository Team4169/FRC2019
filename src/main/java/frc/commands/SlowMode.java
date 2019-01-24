/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;

public class SlowMode extends Command {
  public SlowMode() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  static final double slowModeConstant = 0.7;
  public boolean buttonPush = false;
  
  public void slowMode(double speed) {
    if (buttonPush = true) { //if the button is pressed return true
      speed *= slowModeConstant;
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    buttonPush = !buttonPush;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    buttonPush = !buttonPush;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
