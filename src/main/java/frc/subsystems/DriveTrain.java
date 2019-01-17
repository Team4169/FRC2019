/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.commands.DriveWithController;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
    public static final int TALON_ONE_PORT = 0;
    public static final int TALON_TWO_PORT = 1;

    private static final WPI_TalonSRX talon1 = new WPI_TalonSRX(TALON_ONE_PORT);
    private static final WPI_TalonSRX talon2 = new WPI_TalonSRX(TALON_TWO_PORT);

    public static double leftY = 0;
    public static double rightY = 0;
    public static final double DEAD_ZONE = 0.2;

    static DifferentialDrive drive = new DifferentialDrive(talon1, talon2);


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveWithController());
  }

  // implement this to drive with a controller
  public void drive() {
    leftY = Robot.m_oi.getController().getY(Hand.kLeft);
      if  (Math.abs(leftY) < DEAD_ZONE) {
        leftY = 0;
      }

    rightY = Robot.m_oi.getController().getY(Hand.kRight);
    if  (Math.abs(rightY) < DEAD_ZONE) {
      rightY = 0;
    }

    drive.tankDrive(leftY, rightY);
  }

  public void stop() {
    drive.tankDrive(0, 0);
  }
}
