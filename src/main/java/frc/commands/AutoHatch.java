/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RouteToTarget;
import frc.robot.Robot.AutoHatchState;

public class AutoHatch extends CommandGroup {

  public static final double VELOCITY = 12;

  public AutoHatch() {
    addSequential(new FindTarget());

    addSequential(new setAutoState(AutoHatchState.eIntercept));
    addSequential(new ZeroSensors());
    addSequential(new TurnToAngle());

    addSequential(new DriveStraightForDistance(VELOCITY));
    
    addSequential(new setAutoState(AutoHatchState.eNormal));

    addSequential(new TurnToAngle());

    addSequential(new DriveStraightForDistance(VELOCITY));
    addSequential(new ReleaseHatch());

    addSequential(new setAutoState(AutoHatchState.eBack));

    addSequential(new DriveStraightForDistance(VELOCITY));

    addSequential(new setAutoState(AutoHatchState.eDone));
  }

  public AutoHatch(RouteToTarget route) {
    addSequential(new ZeroSensors());
    addSequential(new TurnToAngle(route.getInterceptVec().getTheta()));

    addSequential(new DriveStraightForDistance(route.getInterceptVec().getR(), VELOCITY));
    
    addSequential(new TurnToAngle(route.getNormalVec().getTheta()));

    addSequential(new DriveStraightForDistance(route.getNormalVec().getR(), VELOCITY));
    addSequential(new ReleaseHatch());

    addSequential(new DriveStraightForDistance(-route.getNormalVec().getR(), VELOCITY));
  }
}
