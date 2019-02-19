/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.commands.commandgroups.AutoHatch;
import frc.robot.Robot;

public class DriveToTarget extends ConditionalCommand {

  public DriveToTarget() {
    super(new AutoHatch(Robot.getCurrentRoute()), new PrintCommand("Route not found"));
  }

  @Override
  protected boolean condition() {
    return (Robot.getCurrentRoute() != null);
  }

}
