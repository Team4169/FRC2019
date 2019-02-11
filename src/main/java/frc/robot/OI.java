/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.commands.DriveStraight;
import frc.commands.DriveWithController;
import frc.commands.SlowMode;
import frc.commands.TurnToAngle;
import frc.commands.ZeroSensors;


/**
 * Add your docs here.
 */
public class OI {

    
    
  private static final XboxController controller1 = new XboxController(RobotMap.CONTROLLER_PORT1);
  private static final XboxController controller2 = new XboxController(RobotMap.CONTROLLER_PORT2);
  private static final JoystickButton BACK_BUTTON1 = new JoystickButton(controller1, RobotMap.BACK_ID);
  private static final JoystickButton Y_BUTTON1 = new JoystickButton(controller1, RobotMap.Y_ID);
  private static final JoystickButton X_BUTTON1 = new JoystickButton(controller1, RobotMap.X_ID);
  private static final JoystickButton B_BUTTON1 = new JoystickButton(controller1, RobotMap.B_ID);
  private static final JoystickButton A_BUTTON1 = new JoystickButton(controller1, RobotMap.A_ID);
  private static final JoystickButton BACK_BUTTON2 = new JoystickButton(controller2, RobotMap.BACK_ID);
  private static final JoystickButton Y_BUTTON2 = new JoystickButton(controller2, RobotMap.Y_ID);
  private static final JoystickButton X_BUTTON2 = new JoystickButton(controller2, RobotMap.X_ID);
  private static final JoystickButton B_BUTTON2 = new JoystickButton(controller2, RobotMap.B_ID);
  private static final JoystickButton A_BUTTON2 = new JoystickButton(controller2, RobotMap.A_ID);

  public OI() {
    BACK_BUTTON2.whenPressed(new ZeroSensors());
    Y_BUTTON1.whenPressed(new SlowMode(false));
    X_BUTTON1.whenPressed(new SlowMode(true));
    B_BUTTON1.whenPressed(new DriveStraight());
  }

  public XboxController getController(int port) {
    switch (port) {
      default:
      case 1:
        return controller1;
      case 2:
        return controller2;
    }
  }
}
