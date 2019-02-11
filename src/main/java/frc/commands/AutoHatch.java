/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Limelight;
import frc.robot.Robot;
import frc.robot.RouteToTarget;
import frc.robot.TargetCalc;
import frc.robot.Vec2D;
import frc.subsystems.DriveTrain;

public class AutoHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoHatch() {
    addSequential(new Extend());
    addSequential(new FindTarget());

    TargetCalc calc = new TargetCalc(Limelight.HEIGHT, Limelight.ANGLE_FROM_HORIZONTAL);
    Vec2D robotVec = new Vec2D(0, 1);
    Vec2D targNorm = new Vec2D(0, -1);
    RouteToTarget route = calc.getRouteToTarget(Robot.ll.getTx(), Robot.ll.getTy(), robotVec, targNorm, Limelight.targetHeight, 12);

    addSequential(new ZeroSensors());
    addSequential(new TurnToAngle(route.getInterceptVec().getTheta()));
    addSequential(new DriveStraightForDistance(route.getInterceptVec().getR()));
    addSequential(new TurnToAngle(route.getNormalVec().getTheta()));
    addSequential(new DriveStraightForDistance(route.getNormalVec().getR()));
    addSequential(new ReleaseHatch());
    addSequential(new DriveStraightForDistance(-route.getNormalVec().getR()));


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
