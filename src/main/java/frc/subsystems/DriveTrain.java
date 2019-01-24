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
    public static final int LEFT_BACK = 1;
    public static final int LEFT_FRONT = 4;
    public static final int RIGHT_BACK = 5;
    public static final int RIGHT_FRONT = 2;


    private static final WPI_TalonSRX leftBack = new WPI_TalonSRX(LEFT_BACK);
    private static final WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT);
    private static final WPI_TalonSRX rightBack = new WPI_TalonSRX(RIGHT_BACK);
    private static final WPI_TalonSRX rightFront = new WPI_TalonSRX(RIGHT_FRONT);
    public static double LeftY = 0;
    public static final SpeedControllerGroup left = new SpeedControllerGroup(leftBack, leftFront);
    public static final SpeedControllerGroup right = new SpeedControllerGroup(rightBack, rightFront);
    public static final DifferentialDrive drive = new DifferentialDrive(left, right);
    public static double RightY = 0;

    @Override
    public void initDefaultCommand() {
    
    
    setDefaultCommand(new DriveWithController()); 
  
    }
    
    //Stars program with slow mode off
    boolean Slow = false;
    //Defines clock. Used to stop double registering of buttons. Currently set to update every half second.
    int clock = 0;

    // Change "zone" to whatever you  want the dead zone to be. "factor" will automaticly scale the code.
    double zone = 0.2;
    double factor = 1 + (1 / (10 * (1-zone)));

    // "SlowFactor" times the speed = slow mode speed. Change SlowFactor to change how slow slowmode is. MAKE SURE THIS IS ALWAYS A DECMAL.
    double SlowFactor = 0.5;



   //implement this to drive with a controller
  public void tankDrive() {
    

    LeftY = Robot.m_oi.getController().getY(Hand.kLeft);
    RightY = Robot.m_oi.getController().getY(Hand.kRight);

   //deadzone

    if (Math.abs(LeftY)< zone) {

      LeftY = 0;

    }
    if (Math.abs(RightY)< zone) {

      RightY = 0;

    }

   //Cheking for slowmode

    if (clock<50) {

     clock += 1;

    }
     else {
       clock = 0;
       boolean CheckSlow = Robot.m_oi.getController().getYButtonPressed();
     if (CheckSlow) {
       Slow=!Slow;
     }
   }
     //Exacuting slow mode

     while (Slow) {

      LeftY = LeftY * SlowFactor;
      RightY = RightY * SlowFactor;

     }

    //Output

    drive.tankDrive(LeftY, RightY);

  }
  public void arcadedrive() {

    double LeftTrig = Robot.m_oi.getController().getTriggerAxis(Hand.kLeft);
    double RightTrig = Robot.m_oi.getController().getTriggerAxis(Hand.kRight);
    double LeftY = Robot.m_oi.getController().getY(Hand.kLeft);
    double Turn = LeftTrig - RightTrig;
    //Checks for deadzone. If not in dead zone, scales. Change deadzone at begining of class using the double "zone". DO NOT CHANGE FACTOR.

    //Checks for positive value.
    if (RightTrig > zone) {
    RightTrig = RightTrig - zone;
    RightTrig = factor * RightTrig;
      if (RightTrig > 1) {
        RightTrig = 1;
      }

    }
    //Checks for negitive value
    else if (RightTrig < 0-zone) {
      RightTrig = RightTrig + zone;
      RightTrig = factor * RightTrig;
        if (RightTrig < -1) {
          RightTrig = -1;
        }

      }
    //Concludes that value is in dead zone. Sets to 0.
    else {

      RightTrig = 0;

     }
     //Output
     drive.arcadeDrive(LeftY, RightTrig);

  }




  //Stops robot when called.
  public void stop() {
    drive.tankDrive (0,0);

  }
  }
