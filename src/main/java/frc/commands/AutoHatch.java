/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot.AutoHatchState;

public class AutoHatch extends CommandGroup {

  public static final double VELOCITY = 12;

  public AutoHatch() {
    addSequential(new CheckRoute());

    addSequential(new setAutoState(AutoHatchState.eIntercept));
    addSequential(new ZeroSensors());
    addSequential(new TurnToAngle());

    addSequential(new ZeroDrive());
    addSequential(new DriveStraightForDistance(VELOCITY));
    
    addSequential(new setAutoState(AutoHatchState.eNormal));

    addSequential(new TurnToAngle());

    addSequential(new ZeroDrive());
    addSequential(new DriveStraightForDistance(VELOCITY));
    addSequential(new ReleaseHatch());

    addSequential(new setAutoState(AutoHatchState.eBack));

    addSequential(new ZeroDrive());
    addSequential(new DriveStraightForDistance(VELOCITY));

    addSequential(new setAutoState(AutoHatchState.eDone));

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
