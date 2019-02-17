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
import frc.commands.ZeroSensors;


/**
 * Add your docs here.
 */
public class OI {

    
    
  private final XboxController controller1 ;
  private final XboxController controller2;
  private final JoystickButton BACK_BUTTON1;
  private final JoystickButton Y_BUTTON1;
  private final JoystickButton X_BUTTON1;
  private final JoystickButton B_BUTTON1;
  private final JoystickButton A_BUTTON1;
  private final JoystickButton BACK_BUTTON2;
  private final JoystickButton Y_BUTTON2;
  private final JoystickButton X_BUTTON2;
  private final JoystickButton B_BUTTON2;
  private final JoystickButton A_BUTTON2;

  public OI() {
    controller1 = new XboxController(RobotMap.CONTROLLER_PORT1);
    controller2 = new XboxController(RobotMap.CONTROLLER_PORT2);

		BACK_BUTTON1 = new JoystickButton(controller1, RobotMap.BACK_ID);
		Y_BUTTON1 = new JoystickButton(controller1, RobotMap.Y_ID);
		X_BUTTON1 = new JoystickButton(controller1, RobotMap.X_ID);
		B_BUTTON1 = new JoystickButton(controller1, RobotMap.B_ID);
		A_BUTTON1 = new JoystickButton(controller1, RobotMap.A_ID);
		BACK_BUTTON2 = new JoystickButton(controller2, RobotMap.BACK_ID);
		Y_BUTTON2 = new JoystickButton(controller2, RobotMap.Y_ID);
		X_BUTTON2 = new JoystickButton(controller2, RobotMap.X_ID);
		B_BUTTON2 = new JoystickButton(controller2, RobotMap.B_ID);
		A_BUTTON2 = new JoystickButton(controller2, RobotMap.A_ID);

    BACK_BUTTON2.whenPressed(new ZeroSensors());
    Y_BUTTON1.whenPressed(new SlowMode(false));
    X_BUTTON1.whenPressed(new SlowMode(true));
    B_BUTTON1.whenPressed(new DriveStraight());
    A_BUTTON1.whenPressed(new DriveWithController()); // Interrupts other commands.
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
