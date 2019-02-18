/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands.commandgroups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.commands.DriveStraightForDistance;
import frc.commands.TurnToAngle;
import frc.commands.ZeroDrive;
import frc.robot.Vec2D;

public class DriveVector extends CommandGroup {
  /**
   * Add your docs here.
   */

  /** Turns to the angle of a vector and then drives the magnitude of the vector */
  public DriveVector(Vec2D vector, double velocity) {
    addSequential(new TurnToAngle(vector.getTheta()));

    addSequential(new ZeroDrive());
    addSequential(new DriveStraightForDistance(vector.getR(), velocity));
  }
}
