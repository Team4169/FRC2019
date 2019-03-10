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

import frc.robot.RobotMap;
// import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */

 
public class Climber extends Subsystem {

  WPI_TalonSRX climber1 = new WPI_TalonSRX(RobotMap.CLIMBER1);
  WPI_TalonSRX climber2 = new WPI_TalonSRX(RobotMap.CLIMBER2);

  //SpeedControllerGroup climber = new SpeedControllerGroup(climber1, climber2);

  private static final double GEARBOX_RATIO = 100.0/1.0;
  private static final int COUNTS_PER_REVOLUTION = 12;
  private static final double TURNING_ANGLE = 132.0;
  private static final double ENCODER_THRESHOLD = 100.0;
  //This is a placeholder value that will have to be calculated later. We want the motor to rotate 132 degrees.
  private static final double TOTAL_CLICKS = GEARBOX_RATIO * ((double) COUNTS_PER_REVOLUTION) * TURNING_ANGLE / 360d;

  private static final double CLIMB_SPEED = 0.5;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Climber() {
    climber1.configFactoryDefault();
    climber2.configFactoryDefault();

    climber1.setInverted(false);
    climber2.setInverted(false);

    climber1.set(ControlMode.PercentOutput, 0);
    climber2.set(ControlMode.PercentOutput, 0);

    climber1.setNeutralMode(NeutralMode.Brake);
    climber2.setNeutralMode(NeutralMode.Brake);

    climber2.follow(climber1);
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public void climb() {
    climber1.set(CLIMB_SPEED);
    SmartDashboard.putNumber("climber1", climber1.getOutputCurrent());
    SmartDashboard.putNumber("climber2", climber2.getOutputCurrent());
  }

  public void unclimb() {
    climber1.set(-CLIMB_SPEED);
  }

  public boolean done() {
    return climber1.getSelectedSensorPosition() >= TOTAL_CLICKS - ENCODER_THRESHOLD;
  }

  public void stop() {
    climber1.set(ControlMode.PercentOutput, 0);
    climber2.set(ControlMode.PercentOutput, 0);
  }

}
