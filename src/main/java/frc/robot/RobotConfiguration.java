package frc.robot;


import edu.wpi.first.wpilibj.XboxController;

/*
 * Stores motors, sensors, and other needed constants that are used more than once
 */ 

public class RobotConfiguration {
    public static final XboxController controller1 = new XboxController(0);
    public static final XboxController controller2 = new XboxController(1);

    public static final int LEFT_FRONT = 4;
    public static final int RIGHT_FRONT = 2;
    public static final int LEFT_BACK = 1;
    public static final int RIGHT_BACK = 5;

}