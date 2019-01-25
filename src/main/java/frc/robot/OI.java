/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.commands.SetDriveType;
import frc.commands.SlowMode;
import frc.subsystems.DriveTrain.DriveType;;

/**
 * Add your docs here.
 */
public class OI {

    public static final int CONTROLLER_PORT = 0;

    public static final int A_ID = 1;
    public static final int B_ID = 2;
    public static final int X_ID = 3;
    
    private static final XboxController controller = new XboxController(CONTROLLER_PORT);
    private static final JoystickButton X_BUTTON = new JoystickButton(controller, X_ID);
    private static final JoystickButton B_BUTTON = new JoystickButton(controller, B_ID);
    private static final JoystickButton A_BUTTON = new JoystickButton(controller, A_ID);
    
    public OI() {
        X_BUTTON.whenPressed(new SlowMode());
        A_BUTTON.whenPressed(new SetDriveType(DriveType.kArcade));
        B_BUTTON.whenPressed(new SetDriveType(DriveType.kStraight));
    }

    public XboxController getController() {
        return controller;
    }
}
