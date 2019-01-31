/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.commands.SlowMode;
import frc.subsystems.Hatch;

/**
 * Add your docs here.
 */
public class OI {

    public static final int CONTROLLER_PORT = 0;
    public static final int Y_ID = 3;
    public static final int RBumper_ID = 5;
    public static final int LBumper_ID = 10;
    private static final XboxController controller1 = new XboxController(CONTROLLER_PORT);
    private static final JoystickButton Y_BUTTON = new JoystickButton(controller1, Y_ID);
    private static final JoystickButton B_BUTTON = new JoystickButton(controller1, B_ID);
    public static final JoystickButton RIGHT_BUMPER = new JoystickButton(controller1, RBumper_ID);
    public static final JoystickButton LEFT_BUMPER = new JoystickButton(controller1, LBumper_ID);

    public OI() {
        Y_BUTTON.whenPressed(new SlowMode());
        RIGHT_BUMPER.whenPressed(new ArmsIn());
        RIGHT_BUMPER.whenPressed(new ArmsOut());
        LEFT_BUMPER.whenPressed(new ArmsIn());
    }

    public XboxController getController() {
        return controller1;
    }
}
