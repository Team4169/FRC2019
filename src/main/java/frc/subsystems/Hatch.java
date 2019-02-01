/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import java.sql.Time;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;

public class Hatch extends Subsystem {

    public static final int TALON_FIVE_PORT = 4;
    public static final int TALON_SIX_PORT = 5;
    public static final int TALON_SEVEN_PORT = 6;

    public static final WPI_TalonSRX extend = new WPI_TalonSRX(TALON_FIVE_PORT);
    public static final WPI_TalonSRX rightArm = new WPI_TalonSRX(TALON_SIX_PORT);
    public static final WPI_TalonSRX leftArm = new WPI_TalonSRX(TALON_SEVEN_PORT);

    public static SpeedControllerGroup arms = new SpeedControllerGroup(leftArm, rightArm);
    public static final double speed = 0.3;

    public static DigitalInput hatchLimitSwitch = new DigitalInput(RobotMap.hatchLimitSwitch);
    

    @Override
    protected void initDefaultCommand() {

    }


    
}