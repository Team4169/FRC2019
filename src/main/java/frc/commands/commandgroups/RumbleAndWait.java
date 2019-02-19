/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.commands.Rumble;
import frc.commands.WaitForButtonPress;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class RumbleAndWait extends CommandGroup {
  /**
   * Add your docs here.
   */

  public static final double RUMBLE_TIME = 0.5;

  public RumbleAndWait() {
    addParallel(new Rumble(), RUMBLE_TIME);
    addParallel(new WaitForButtonPress(Robot.m_oi.getController(1), RobotMap.RIGHT_JOY_ID));
  }
}
