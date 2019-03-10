/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.commands.ReleaseForTime;
import frc.commands.ZeroArmsReleased;

public class ReleaseAndZero extends CommandGroup {
  /**
   * Add your docs here.
   */

	static final double TIME_FOR_RELEASE = 0.5;
	
  public ReleaseAndZero() {
    addSequential(new ReleaseForTime(TIME_FOR_RELEASE));
    addSequential(new ZeroArmsReleased());
  }
}
