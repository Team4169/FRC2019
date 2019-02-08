/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.commands.Climb;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */

 
public class Climber extends Subsystem {

  WPI_TalonSRX climber1 = new WPI_TalonSRX(RobotMap.CLIMBER1);
  WPI_TalonSRX climber2 = new WPI_TalonSRX(RobotMap.CLIMBER2);

  private static final double totalClicks = 9000; //This is a placeholder value that will have to be calculated later. We want the motor to rotate 132 degrees.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new Climb());
  }

  public Climber() {

    climber1.configFactoryDefault();
    climber2.configFactoryDefault();

    climber1.set(ControlMode.PercentOutput, 0);
    climber2.set(ControlMode.PercentOutput, 0);

    climber1.setNeutralMode(NeutralMode.Brake);
    climber2.setNeutralMode(NeutralMode.Brake);

    climber2.follow(climber1);

  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void climb() {
    climber1.set(0.5);
  }

  public boolean done() {
    if (climber1.getSelectedSensorPosition() > totalClicks) return true;
    else return false;
  }

  public void stop() {
    climber1.set(ControlMode.PercentOutput, 0);
    climber2.set(ControlMode.PercentOutput, 0);
  }

}
