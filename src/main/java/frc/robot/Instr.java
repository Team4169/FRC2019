/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Instrumentation package
 * Periodically display useful information to the smart dashboard.
 * Centralized for easy maintenance.  Called on every robotPeriodic()
 * (i.e. 20ms) but only updates the display every DISPLAY_COUNT times 
 * to avoid excessive overhead.
 */
public class Instr {

    public static int DISPLAY_COUNT = 10;   // update display 5x per sec

    /**
     * Call count since last display
     */
    private int execCount;
    private PowerDistributionPanel pdp;

    /**
     * Don't update the display during the constructor; we
     * don't know that all needed classes have been
     * initialized yet.  Wait until first 
     */
    public Instr() {
        execCount = DISPLAY_COUNT - 1;
        pdp = new PowerDistributionPanel();
     }

    /**
     * Called on each robotPeriodic(); update the display
     * if it's time to.
     */
    public void periodic() {
        if (++execCount >= DISPLAY_COUNT) {
            execCount = 0;
            updateDisplay();
        }
    }

    /**
     * Update the display of all instrumentation
     */
    private void updateDisplay() {
        try {
            SmartDashboard.putNumber("Yaw", Robot.kDriveTrain.getYaw());
            SmartDashboard.putNumber("Angle", Robot.kDriveTrain.getAngle());
            SmartDashboard.putNumber("Left distance", Robot.kDriveTrain.getLeftDistance());
            SmartDashboard.putNumber("Left rate", Robot.kDriveTrain.getLeftRate());
            SmartDashboard.putNumber("LeftBack I", pdp.getCurrent(RobotMap.PDP_LB));
            SmartDashboard.putNumber("LeftFront I", pdp.getCurrent(RobotMap.PDP_LF));
            SmartDashboard.putNumber("Right distance", Robot.kDriveTrain.getRightDistance());
            SmartDashboard.putNumber("Right rate", Robot.kDriveTrain.getRightRate());
            SmartDashboard.putNumber("RightBack I", pdp.getCurrent(RobotMap.PDP_RB));
            SmartDashboard.putNumber("RightFront I", pdp.getCurrent(RobotMap.PDP_RF));
            SmartDashboard.putNumber("Voltage", pdp.getVoltage());
            SmartDashboard.putBoolean("Brownout", RobotController.isBrownedOut());
        } catch (Exception ex) {
            System.err.println("Instr caught exception " + ex);
        }
    }
}
