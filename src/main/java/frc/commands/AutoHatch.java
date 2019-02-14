/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.RouteToTarget;

public class AutoHatch extends CommandGroup {

  public static final double VELOCITY = 12;

  public AutoHatch() {
    RouteToTarget route = Robot.getCurrentRoute();

    addSequential(new ZeroSensors());
    addSequential(new TurnToAngle(route.getInterceptVec().getTheta()));

    addSequential(new ZeroDrive());
    addSequential(new DriveStraightForDistance(route.getInterceptVec().getR(), VELOCITY));
    
    addSequential(new TurnToAngle(route.getNormalVec().getTheta()));

    addSequential(new ZeroDrive());
    addSequential(new DriveStraightForDistance(route.getNormalVec().getR(), VELOCITY));
    addSequential(new ReleaseHatch());

    addSequential(new ZeroDrive());
    addSequential(new DriveStraightForDistance(-route.getNormalVec().getR(), VELOCITY));
    // TODO possible recursion? ^^^^

    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
