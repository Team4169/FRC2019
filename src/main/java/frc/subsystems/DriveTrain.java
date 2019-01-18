/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
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
    public static final int TALON_THREE_PORT = 2;
    public static final int TALON_FOUR_PORT = 3;
    

    private static final WPI_TalonSRX talon1 = new WPI_TalonSRX(TALON_ONE_PORT);
    private static final WPI_TalonSRX talon2 = new WPI_TalonSRX(TALON_TWO_PORT);
    private static final WPI_TalonSRX talon3 = new WPI_TalonSRX(TALON_THREE_PORT);
    private static final WPI_TalonSRX talon4 = new WPI_TalonSRX(TALON_FOUR_PORT);
    public static double LeftY = 0;
    public static final SpeedControllerGroup Group1 = new SpeedControllerGroup(talon1, talon3);
    public static final SpeedControllerGroup Group2 = new SpeedControllerGroup(talon2, talon4);
    public static final DifferentialDrive drive = new DifferentialDrive(Group1, Group2);
    public static double RightY = 0;
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new DriveWithController());
  }

  // implement this to drive with a controller
  public void tankDrive() {
    
    LeftY = Robot.m_oi.getController().getY(Hand.kLeft);
    RightY = Robot.m_oi.getController().getY(Hand.kRight);
    if (Math.abs(LeftY)< 0.2) {

      LeftY = 0;

    }
    if (Math.abs(RightY)< 0.2) {

      RightY = 0;

    }
    drive.tankDrive(LeftY, RightY);


  }
  public void arcadedrive() {

    double LeftTrig = Robot.m_oi.getController().getTriggerAxis(Hand.kLeft);
    double RightTrig = Robot.m_oi.getController().getTriggerAxis(Hand.kRight);
    double LeftY = Robot.m_oi.getController().getY(Hand.kLeft);
    double Turn = LeftTrig - RightTrig;
    if (Math.abs(LeftTrig)< 0.2) {

      LeftTrig = 0;

    }
    if (Math.abs(RightTrig)< 0.2) {

      RightTrig = 0;

    }
    if (RightTrig > 0.2) {
    RightTrig = RightTrig - 0.2;
    RightTrig = 1.125 * RightTrig;
      if (RightTrig > 1) {
        RightTrig = 1;
      }

    }
    drive.arcadeDrive(LeftY, Turn);


  }
  public void stop() {
    drive.tankDrive (0,0);

  }
  }