/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.subsystems.DriveTrain;

public class DriveStraightForDistance extends Command {

  double distance;

  // only for auto step
  double velocity;

  boolean finished;
  int accelSteps;
  int runSteps, nSteps, nAccelSteps, nRunSteps, nDecelSteps;

  boolean triangularAccel;

  enum RunState {
    eAccel, eRun, eDecel, eDone
  }

  RunState runState;

  // for auto:
  Robot.AutoStep step = null;

  public DriveStraightForDistance(double distance, double velocity) {
    requires(Robot.kDriveTrain);
    setup(distance, velocity);
    this.velocity = 0; // if the program breaks ("I probably wrote it wrong" - Stu)
  }

  public DriveStraightForDistance(Robot.AutoStep a, double vel) {
    requires(Robot.kDriveTrain);
    step = a;
    velocity = vel;
    this.distance = 0; // if the program breaks ("I probably wrote it wrong" - Stu)
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if (step != null) {
      double dist;
      switch(step) {
      case kIntercept:
        dist = Robot.getCurrentRoute().getInterceptVec().getR();
        break;
      default:
      case kApproach:
        dist = Robot.getCurrentRoute().getNormalVec().getR();
        break;
      }

      setup(dist, velocity);
    }

    if (finished) {
      System.out.println("Drive straight for distance failed - velocity not legitimate");
      end();
    } else {
      Robot.kDriveTrain.driveStraightFirstCall();
      Robot.kDriveTrain.zeroDriveEncoders();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double motorPower;
    if (nSteps == 0) {
      motorPower = DriveTrain.startPower;
    } else {
      motorPower = Robot.kDriveTrain.getCurrentPower();
    }

    nSteps++;
    if (runState == RunState.eAccel) {
      nAccelSteps++;
      motorPower += DriveTrain.powerPerStep;
      if (motorPower > 1.0) {
        motorPower = 1.0;
      }
      if (nAccelSteps >= accelSteps) {
        runState = RunState.eRun;
      }
    } else if (runState == RunState.eRun) {
      nRunSteps++;
      if (nRunSteps >= runSteps) {
        runState = RunState.eDecel;
      }
    } else if (runState == RunState.eDecel) {
      nDecelSteps++;
      if (motorPower > DriveTrain.startPower + DriveTrain.powerPerStep) {
        motorPower -= DriveTrain.powerPerStep;
      }
    } else if (runState == RunState.eDone) {
      motorPower = 0.0;
    }

    double dist = Robot.kDriveTrain.getCurrentDistance();
    Robot.kDriveTrain.driveStraight(motorPower);

    System.out.println("exec state " + runState.toString() + " setting power to " + motorPower + " at dist " + dist);
  }

  

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished || Robot.kDriveTrain.getCurrentDistance() >= distance;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.kDriveTrain.disableTurnController();
    Robot.kDriveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }

  public void setup(double distance, double velocity) {
    this.distance = distance;
    finished = (velocity <= 0 || velocity > DriveTrain.maxVelocity);

    // assume trapezoid
    accelSteps = (int) Math.ceil(velocity / DriveTrain.velocityPerStep);

    double accelDistance = Robot.kDriveTrain.calculateAccelDistance(accelSteps, DriveTrain.velocityPerStep, DriveTrain.secPerStep);

    if (2d * accelDistance < distance) {
      triangularAccel = false;
    } else {
      triangularAccel = true;
      
      accelDistance = Math.floor(distance / 2d);
      accelSteps = (int) Robot.kDriveTrain.calculateAccelSteps(accelDistance, DriveTrain.velocityPerStep, DriveTrain.secPerStep);
      accelDistance = Robot.kDriveTrain.calculateAccelDistance(accelSteps, DriveTrain.velocityPerStep, DriveTrain.secPerStep);
    }

    double runVelocity = accelSteps * DriveTrain.velocityPerStep;
    double runDistance = distance - (2 * accelDistance);
    double runTime = runDistance / runVelocity;
    runSteps = (int) Math.floor(runTime / DriveTrain.secPerStep);
    System.out.println("Triangular acceleration: " + Boolean.toString(triangularAccel));
    System.out.println("rdist " + runDistance + " rtime " + runTime + " rsteps " + runSteps);
    System.out.println("accSteps " + accelSteps + " accDist " + accelDistance);
    nSteps = 0;
    nAccelSteps = 0;
    nRunSteps = 0;
    nDecelSteps = 0;
    runState = RunState.eAccel;
  }
}
