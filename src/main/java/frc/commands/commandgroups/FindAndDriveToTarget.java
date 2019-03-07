/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.commands.commandgroups.DriveToTarget;
import frc.commands.FindTarget;

public class FindAndDriveToTarget extends CommandGroup {
  /**
   * Add your docs here.
   */

  public FindAndDriveToTarget() {
    addSequential(new FindTarget());
    addSequential(new RumbleAndWait());
    addSequential(new DriveToTarget());
  }
}
