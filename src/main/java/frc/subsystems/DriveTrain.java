/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.commands.DriveWithController;
import frc.commands.SlowMode;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
    public static final int TALON_ONE_PORT = 0;
    public static final int TALON_TWO_PORT = 1;
    public static final int TALON_THREE_PORT = 2;
    public static final int TALON_FOUR_PORT = 3;

    private static final WPI_TalonSRX leftFront = new WPI_TalonSRX(TALON_ONE_PORT);
    private static final WPI_TalonSRX rightFront = new WPI_TalonSRX(TALON_TWO_PORT);
    private static final WPI_TalonSRX leftBack = new WPI_TalonSRX(TALON_THREE_PORT);
    private static final WPI_TalonSRX rightBack = new WPI_TalonSRX(TALON_FOUR_PORT);

    public static SpeedControllerGroup left = new SpeedControllerGroup(leftFront, leftBack);
    public static SpeedControllerGroup right = new SpeedControllerGroup(rightFront, rightBack);

    public static final double DEAD_ZONE = 0.2;
    public static final double slowModeConstant = 0.7;
    public static boolean buttonPush = false;

    private static double deadZone(double current) {
      if (Math.abs(current) < DEAD_ZONE) {
        return 0;
      } else if (current > 0) {
        return (4 * current/5 + DEAD_ZONE);
      } else {
        return (4 * current / 5 + DEAD_ZONE);
      }
    }

    private double slowMode(double number) {
      if (buttonPush = true) { //if the button is pressed return true
        number *= slowModeConstant;
      }
    return number;
    }

    static DifferentialDrive drive = new DifferentialDrive(left, right);
    static SlowMode slowMode = new SlowMode();

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveWithController());
  }

  // implement this to drive with a controller
  public void tankDrive() {
    double leftY = Robot.m_oi.getController().getY(Hand.kLeft);;
    double rightY = Robot.m_oi.getController().getY(Hand.kRight);;

    leftY = deadZone(leftY);
    rightY = deadZone(rightY);

    drive.tankDrive(leftY, rightY);
  }

  public void arcadeDrive() {
    double speed = Robot.m_oi.getController().getY(Hand.kLeft);
    double leftTrigger = Robot.m_oi.getController().getTriggerAxis(Hand.kLeft);
    double rightTrigger = Robot.m_oi.getController().getTriggerAxis(Hand.kRight);
  
    double rotation;
    rotation = rightTrigger - leftTrigger;

    speed = deadZone(speed);
    speed = slowMode(speed);
    drive.arcadeDrive(speed, rotation);
  }


public void stop() {
    drive.tankDrive(0,0); 
  }
}
