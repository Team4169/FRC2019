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

public class ExtendAndGrab extends CommandGroup {
  /**
   * Add your docs here.
   */

  public static final double TIME_TO_GRAB = 1.0;
  
  public ExtendAndGrab() {
    addSequential(new Extend());
    addSequential(new GrabHatch());
  }
}
