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
import frc.commands.SwitchDriveType;
import frc.commands.ZeroSensors;
import frc.subsystems.DriveTrain;
import frc.commands.GrabHatch;
import frc.commands.ReleaseHatch;

/**
 * Add your docs here.
 */
public class OI {

    
    
  private static final XboxController controller1 = new XboxController(RobotMap.CONTROLLER_PORT);
  private static final JoystickButton BACK_BUTTON = new JoystickButton(controller1, RobotMap.BACK_ID);
  private static final JoystickButton Y_BUTTON = new JoystickButton(controller1, RobotMap.Y_ID);
  private static final JoystickButton X_BUTTON = new JoystickButton(controller1, RobotMap.X_ID);
  private static final JoystickButton B_BUTTON = new JoystickButton(controller1, RobotMap.B_ID);
  private static final JoystickButton A_BUTTON = new JoystickButton(controller1, RobotMap.A_ID);
  private static final JoystickButton RB_BUTTON = new JoystickButton(controller1, RobotMap.RB_ID);
  private static final JoystickButton LB_BUTTON = new JoystickButton(controller1, RobotMap.LB_ID);

  public OI() {
    BACK_BUTTON.whenPressed(new ZeroSensors());
    Y_BUTTON.whenPressed(new SlowMode(false));
    X_BUTTON.whenPressed(new SlowMode(true));
    B_BUTTON.whenPressed(new SwitchDriveType(DriveTrain.DriveType.kDriveStraight));
    A_BUTTON.whenPressed(new SwitchDriveType(DriveTrain.DriveType.kArcade));
    RB_BUTTON.whenPressed(new GrabHatch());
    LB_BUTTON.whenPressed(new ReleaseHatch());
  }

  public XboxController getController() {
    return controller1;
  }
}
