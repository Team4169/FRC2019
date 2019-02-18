/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.commands.ZeroArms;

public class InitialCommand extends CommandGroup {
  /**
   * Add your docs here.
   */
  public InitialCommand() {
    addSequential(new ExtendAndGrab());
    addSequential(new ZeroArms());
  }
}
