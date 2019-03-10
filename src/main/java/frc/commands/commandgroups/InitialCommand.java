/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.commands.Extend;
import frc.commands.GrabHatch;
import frc.commands.ZeroDrive;

public class InitialCommand extends CommandGroup {

  public InitialCommand() {
    addSequential(new ZeroDrive());
    addSequential(new Extend());
    addSequential(new ReleaseAndZero());
    addSequential(new GrabHatch());
  }
}
