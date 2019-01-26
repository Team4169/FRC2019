/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//this is the main code here

public class Hatch extends Subsystem {

    public static final int TALON_FIVE_PORT = 4;
    public static final int TALON_SIX_PORT = 5;
    public static final int TALON_SEVEN_PORT = 6;

    private static final WPI_TalonSRX extend = new WPI_TalonSRX(TALON_FIVE_PORT);
    private static final WPI_TalonSRX rightOut = new WPI_TalonSRX(TALON_SIX_PORT);
    private static final WPI_TalonSRX leftOut = new WPI_TalonSRX(TALON_SEVEN_PORT);

    @Override
    protected void initDefaultCommand() {

    }

    
} 




