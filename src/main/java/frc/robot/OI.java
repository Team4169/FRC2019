/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class OI {

    public static final int CONTROLLER_PORT = 0;
    private static final XboxController controller1 = new XboxController(CONTROLLER_PORT);

    public XboxController getController() {
        return controller1;
    }
}
