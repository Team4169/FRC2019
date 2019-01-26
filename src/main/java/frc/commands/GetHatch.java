/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;

public class GetHatch extends Command {
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
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
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
/*  so this should check if the limit swith is pushed. If the limit switch is not 
pushed: then the extend motor should run forward until the limit switch is pushed. 
        then once the switch is pushed it should run the motors for the leftOut 
        and rightOut, to grab the hatch, that will be it and the command will end. 
If the limit switch is pushed then:
        the leftOut and rightOut will retracht (hypothetically putting the hatch 
        on the thing) then the extend will run backwards for (either a period of 
        time or a distance, most likely the first one) then the command ends

*/