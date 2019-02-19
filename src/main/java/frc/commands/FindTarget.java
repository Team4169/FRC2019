package frc.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Limelight.CameraMode;
import frc.robot.Limelight.LightMode;

public class FindTarget extends Command {
  public FindTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.kDriveTrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.ll.setCameraMode(CameraMode.eVision);
    Robot.ll.setLedMode(LightMode.eOn);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.kDriveTrain.findTarget();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.getCurrentRoute() != null || isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.kDriveTrain.stop();
    Robot.ll.setCameraMode(CameraMode.eDriver);
    Robot.ll.setLedMode(LightMode.eOff);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}