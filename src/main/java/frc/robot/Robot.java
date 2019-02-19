/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.DriveTrain;
import frc.subsystems.Hatch;
import frc.commands.commandgroups.InitialCommand;
import frc.subsystems.Climber;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final DriveTrain kDriveTrain = new DriveTrain();
  public static final Hatch kHatch = new Hatch();
  public static final Climber kClimber = new Climber();
  public static final OI m_oi = new OI();
  public static final Limelight ll = new Limelight();
  Command autoCommand;

  static RouteToTarget curRoute;
  public static final int cameraYThreshold = 17;
  public static final Vec2D[] TARG_VECS = new Vec2D[] {
    Vec2D.makeCart(-1, 0), Vec2D.makeCart(0, -1), Vec2D.makeCart(1, 0), Vec2D.makeCart(0, 1)
  };
  public static final double targBoundary = 30;
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("kP", 3.0);
		SmartDashboard.putNumber("kD", 4.0);

    SmartDashboard.putData("Drive Train", kDriveTrain);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */

@Override
	public void teleopInit(){
	
	}

  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autoCommand = new InitialCommand();
    autoCommand.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    ll.setLedMode(Limelight.LightMode.eOn);
    // System.out.println("tx: " + ll.getTx() + ", ty: " + ll.getTy());

    TargetCalc targetCalc = new TargetCalc(ll);
    Vec2D robotVec = Vec2D.makePolar(1, 90);
    Vec2D normalVec = Vec2D.makePolar(1, 270);
    double normalDist = 12;
    RouteToTarget route = targetCalc.getRouteToTarget(robotVec, normalVec, normalDist);
    
    if (m_oi.getController(1).getAButtonPressed()) {
      System.out.println("tx: " + ll.getTx() + ", ty: " + ll.getTy());
      System.out.println("Target Vector: <" + route.getTargetDirectVec().getXCoord() + ", " + route.getNormalVec() + ">");
      System.out.println("Intecept Vector: <" + route.getInterceptVec().getXCoord() + ", " + route.getInterceptVec() + ">");
      System.out.println("Normal Vector: <" + route.getNormalVec().getXCoord() + ", " + route.getNormalVec().getYCoord() + ">");
    }
  }

  public static RouteToTarget getCurrentRoute() {
    return curRoute;
  }

  public static void checkRoute() {
    if (ll.isTarget() && ll.getTy() > -cameraYThreshold) { // TODO
      TargetCalc calc = new TargetCalc(ll);
      Vec2D robotVec = getCurrentRobotVec();
      Vec2D targNorm;

      if (robotVec.getTheta() > -targBoundary && robotVec.getTheta() < targBoundary) {
        targNorm = TARG_VECS[0];
      } else if (robotVec.getTheta() > 90.0 - targBoundary && robotVec.getTheta() < 90.0 + targBoundary) {
        targNorm = TARG_VECS[1];
      } else if (robotVec.getTheta() > 180.0 - targBoundary && robotVec.getTheta() < 180.0 + targBoundary) {
        targNorm = TARG_VECS[2];
      } else if (robotVec.getTheta() > 270.0 - targBoundary && robotVec.getTheta() < 270.0 + targBoundary) {
        targNorm = TARG_VECS[3];
      }
      double normalDist = 12.0; // TODO
      curRoute = calc.getRouteToTarget(robotVec, targNorm, normalDist);
    } else {
      curRoute = null;
    }
  }

  public static Vec2D getCurrentRobotVec() {
    return Vec2D.makePolar(1, kDriveTrain.getYaw());
  }
}
