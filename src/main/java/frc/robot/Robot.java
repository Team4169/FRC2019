/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsystems.DriveTrain;
import frc.subsystems.Hatch;
import frc.commands.Rumble;
// import frc.commands.commandgroups.FindAndDriveToTarget;
import frc.commands.commandgroups.InitialCommand;
// import frc.subsystems.Climber;

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
  //public static final Climber kClimber = new Climber();
  public static final OI m_oi = new OI();
  public static final Limelight ll = new Limelight();
  public static final Instr kInstr = new Instr();

  Command autoCommand;

  boolean firstRumble = false;
  static final Command rumbleCommand = new Rumble(1.0);

  static RouteToTarget curRoute;
  public static final int cameraYThreshold = 17;
  public static final Vec2D[] TARG_VECS = new Vec2D[] {
    Vec2D.makeCart(-1, 0), Vec2D.makeCart(0, -1), Vec2D.makeCart(1, 0), Vec2D.makeCart(0, 1)
  };
  public static final double targBoundary = 30.0; // between 0 and 45
  public static final double NORMAL_DIST = 12.0;


  public enum AutoStep {
    kIntercept, kApproach
  }
  
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putNumber("kP", 3.0);
		SmartDashboard.putNumber("kD", 4.0);

    SmartDashboard.putData("Drive Train", kDriveTrain);

    ll.setLedMode(Limelight.LightMode.eOff);
    SmartDashboard.putNumber("climber1", 0);
    SmartDashboard.putNumber("climber2", 0);
    firstRumble = true;
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
    firstRumble = true;
	}

  @Override
  public void robotPeriodic() {
    kInstr.periodic();
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
    ll.setLedMode(Limelight.LightMode.eOff);

    firstRumble = true;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    checkPOV(); // TODO
    try {
      Scheduler.getInstance().run();
    } catch (Exception e) {
      System.out.println(e.toString());
    }

    if (!kHatch.getLimitSwitch()) {
      if (firstRumble) rumbleCommand.start();
      firstRumble = false;
    } else {
      if (rumbleCommand.isRunning()) rumbleCommand.cancel();
      firstRumble = true;
    }
  }


  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    checkPOV();
    Scheduler.getInstance().run();
    System.out.println("Arm motor encoder: " + kHatch.getEncoderValue());
    if (!kHatch.getLimitSwitch()) {
      if (firstRumble) rumbleCommand.start();
      firstRumble = false;
    } else {
      if (rumbleCommand.isRunning()) rumbleCommand.cancel();
      firstRumble = true;
    }
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    // ll.setLedMode(Limelight.LightMode.eOn);
    // // System.out.println("tx: " + ll.getTx() + ", ty: " + ll.getTy());

    // TargetCalc targetCalc = new TargetCalc(ll);
    // Vec2D robotVec = Vec2D.targetCalc.getRouteToTarget(robotVec, normalVec, normalDist);
    
    // if (m_oi.getController(1).getAButtonPressed()) {
    //   System.out.println("tx: " + ll.getTx() + ", ty: " + ll.getTy());
    //   System.out.println("Target Vector: <" + route.getTargetDirectVec().getXCoord() + ", " + route.getNormalVec() + ">");
    //   System.out.println("Intecept Vector: <" + route.getInterceptVec().getXCoord() + ", " + route.getInterceptVec() + ">");
    //   System.out.println("Normal Vector: <" + route.getNormalVec().getXCoord() + ", " + route.getNormalVec().getYCoord() + ">");
    // }

    System.out.println(kDriveTrain.getCurrentDistance());
    // m_oi.getController().setRumble(RumbleType.kLeftRumble, 1.0);
    // m_oi.getController().setRumble(RumbleType.kRightRumble, 1.0);
  }

  @Override
  public void disabledInit() {
    m_oi.getController().setRumble(RumbleType.kLeftRumble, 0.0);
    m_oi.getController().setRumble(RumbleType.kRightRumble, 0.0);
    ll.setLedMode(Limelight.LightMode.eOff);
    Scheduler.getInstance().removeAll();
  }

  public static RouteToTarget getCurrentRoute() {
    return curRoute;
  }

  public static void checkRoute() {
    if (ll.isTarget() && ll.getTy() > -cameraYThreshold) { // TODO threshold
      TargetCalc calc = new TargetCalc(ll);
      Vec2D robotVec = getCurrentRobotVec();
      Vec2D targNorm = null;

      // TODO check if this makes sense:
      if (robotVec.getTheta() > -targBoundary && robotVec.getTheta() < targBoundary) {
        targNorm = TARG_VECS[0];
      } else if (robotVec.getTheta() > 90.0 - targBoundary && robotVec.getTheta() < 90.0 + targBoundary) {
        targNorm = TARG_VECS[1];
      } else if (robotVec.getTheta() > 180.0 - targBoundary && robotVec.getTheta() < 180.0 + targBoundary) {
        targNorm = TARG_VECS[2];
      } else if (robotVec.getTheta() > 270.0 - targBoundary && robotVec.getTheta() < 270.0 + targBoundary) {
        targNorm = TARG_VECS[3];
      }

      if (targNorm != null) {
        curRoute = calc.getRouteToTarget(robotVec, targNorm, NORMAL_DIST);
      } else {
        curRoute = null;
      }
    } else {
      curRoute = null;
    }
  }

  public static Vec2D getCurrentRobotVec() {
    return Vec2D.makePolar(1, yawToFieldAngle(kDriveTrain.getYaw()));
  }

  /**
   * Convert robot yaw from the compass (in degrees, 0 meaning pointing along the Y axis
   * field-relative) to a field relative angle in degrees (0 meaning pointing along the X axis)
   * Result will be in the range -180..180
   * @param yaw Yaw in degrees, -180..180
   * @return Field angle in degrees
   */
  public static double yawToFieldAngle(double yaw) {
    double fieldAngle = 90.0 - yaw;

    if (fieldAngle > 180.0) {
      fieldAngle -= 360.0;
    } else if (fieldAngle < -180.0) {
      fieldAngle += 360.0;
    }

    return fieldAngle;
  }

  /**
   * Convert robot yaw from the field relative angle in degrees (0 meaning pointing along the X axis)
   * to a compass (in degrees, 0 meaning pointing along the Y axis field-relative)
   * Result will be in the range -180..180
   * @param field Field angle in degrees -180..180
   * @return Yaw in degrees
   */
  public static double fieldToYawAngle(double field) {
    double yaw = 90.0 - field;

    if (yaw > 180.0) {
      yaw -= 360.0;
    } else if (yaw < -180.0) {
      yaw += 360.0;
    }

    return yaw;
  }

  public void checkPOV() {
    if (m_oi.getController().getPOV() == 270) {
      //new FindAndDriveToTarget().start();
      System.out.println("NANANANANANANANANANANANANNANANANANANANANANA");
    }
  }
}
