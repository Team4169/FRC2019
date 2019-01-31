/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import java.sql.Time;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Timer;

public class Hatch extends Subsystem {

    public static final int TALON_FIVE_PORT = 4;
    public static final int TALON_SIX_PORT = 5;
    public static final int TALON_SEVEN_PORT = 6;

    private static final WPI_TalonSRX extend = new WPI_TalonSRX(TALON_FIVE_PORT);
    private static final WPI_TalonSRX right = new WPI_TalonSRX(TALON_SIX_PORT);
    private static final WPI_TalonSRX left = new WPI_TalonSRX(TALON_SEVEN_PORT);

    static DifferentialDrive arms = new DifferentialDrive(left, right);
    
    

    @Override
    protected void initDefaultCommand() {

    }


    private static double endTime = currentTime + 100;
    private static final double speed = 0.3;


    public void extendHatch() {
        double extendTime = currentTime + 50;
        while (currentTime < extendTime) {
            extned.setSpeed(speed);
        }
    }

    public static void ArmsOut() {
        while (endTime > currentTime) {
          right.setSpeed(-speed);
          left.setSpeed(speed);
          double currentTime = getMatchTime();
        }

    }

    public static void ArmsIn() {
        while (endTime > currentTime) {
        right.setSpeed(speed);
        left.setSpeed(-speed);
        double currentTime = getMatchTime();
        }
    }


    
}
