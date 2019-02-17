/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ExtendAndZero extends CommandGroup {
  /**
   * Add your docs here.
   */

  public static final double TIME_TO_GRAB = 0.1;
  
  public ExtendAndZero() {
    addParallel(new Extend());
    addSequential(new GrabHatch(), TIME_TO_GRAB);
    addSequential(new ZeroSensors());
  }
}
