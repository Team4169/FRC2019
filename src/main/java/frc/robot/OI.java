/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.commands.Climb;
// import frc.commands.DriveWithController;
import frc.commands.Extend;
import frc.commands.GrabHatch;
import frc.commands.ReleaseHatch;
import frc.commands.SlowMode;
import frc.commands.Unclimb;
import frc.commands.commandgroups.ReleaseAndZero;


/**
 * Add your docs here.
 */
public class OI {

    
    
  private final XboxController controller1;

  private final JoystickButton BACK_BUTTON1;
  private final JoystickButton Y_BUTTON1;
  private final JoystickButton X_BUTTON1;
  private final JoystickButton B_BUTTON1;
  private final JoystickButton A_BUTTON1;
  private final JoystickButton START_BUTTON1;

  private final JoystickButton LEFT_BUMPER1;
  private final JoystickButton RIGHT_BUMPER1;


  public OI() {
    controller1 = new XboxController(RobotMap.CONTROLLER_PORT1);

    START_BUTTON1 = new JoystickButton(controller1, RobotMap.START_ID);
		BACK_BUTTON1 = new JoystickButton(controller1, RobotMap.BACK_ID);
		Y_BUTTON1 = new JoystickButton(controller1, RobotMap.Y_ID);
		X_BUTTON1 = new JoystickButton(controller1, RobotMap.X_ID);
		B_BUTTON1 = new JoystickButton(controller1, RobotMap.B_ID);
    A_BUTTON1 = new JoystickButton(controller1, RobotMap.A_ID);
    RIGHT_BUMPER1 = new JoystickButton(controller1, RobotMap.RB_ID);
    LEFT_BUMPER1 = new JoystickButton(controller1, RobotMap.LB_ID);

    Y_BUTTON1.whenPressed(new SlowMode(false));
    X_BUTTON1.whenPressed(new SlowMode(true));
    //B_BUTTON1.whenPressed(new DriveStraight());
    B_BUTTON1.whenPressed(new Extend());
    RIGHT_BUMPER1.whenPressed(new GrabHatch());
    LEFT_BUMPER1.whenPressed(new ReleaseHatch());
    // A_BUTTON1.whenPressed(new DriveWithController()); // Interrupts other commands.
    // START_BUTTON1.whileHeld(new Climb());
    // BACK_BUTTON1.whileHeld(new Unclimb());
    A_BUTTON1.whenPressed(new ReleaseAndZero());
  }

  public XboxController getController() {
    return controller1;
  }


}
