/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.commands.ArmsDrop;
import frc.commands.HatchExtend;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;

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
    public static boolean hatchLimitSwitchStatus = hatchLimitSwitch.get();
    

    @Override
    protected void initDefaultCommand() {

    }

    public static final double gameTimeConstant = 100.0;
    public static double matchTime = Timer.getMatchTime();
    public static double startTime = matchTime + gameTimeConstant;
    public static void checkForHatchExtension() {
        if (matchTime < startTime) {
            new HatchExtend();
        }
    }
    public static int LeftBumper = RobotMap.LB_ID;
    public static boolean getBumperPressed(GenericHID.Hand Left) = leftBumperStatus;
    public static void dropHatch() {
        if (leftBumperStatus == true) {
            new ArmsDrop();
        }
    }

    public static void grabHatch() {
hog
    }
}
