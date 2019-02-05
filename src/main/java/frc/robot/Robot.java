package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import java.math.*;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * This is a demo program showing the use of the navX MXP to implement
 * the "rotate to angle", "zero yaw" and "drive straight" on a Tank
 * drive system.
 *
 * If Left Joystick Button 0 is pressed, a "turn" PID controller will 
 * set to point to a target angle, and while the button is held the drive
 * system will rotate to that angle (NOTE:  tank drive systems cannot simultaneously
 * move forward/reverse while rotating).
 *
 * This example also includes a feature allowing the driver to "reset"
 * the "yaw" angle.  When the reset occurs, the new gyro angle will be
 * 0 degrees.  This can be useful in cases when the gyro drifts, which
 * doesn't typically happen during a FRC match, but can occur during
 * long practice sessions.
 *
 * Finally, if Left Joystick button 2 is held, the "turn" PID controller will
 * be set to point to the current heading, and while the button is held,
 * the driver system will continue to point in the direction.  The robot 
 * can drive forward and backward (the magnitude of motion is the average
 * of the Y axis values on the left and right joysticks).
 *
 * Note that the PID Controller coefficients defined below will need to
 * be tuned for your drive system.
 */

public class Robot extends SampleRobot implements PIDOutput {
    DifferentialDrive myRobot;  // class that handles basic drive operations

    AHRS ahrs;

    PIDController turnController;
    double rotateToAngleRate;
    
    /* The following PID Controller coefficients will need to be tuned */
    /* to match the dynamics of your drive system.  Note that the      */
    /* SmartDashboard in Test mode has support for helping you tune    */
    /* controllers by displaying a form where you can enter new P, I,  */
    /* and D constants and test the mechanism.                         */
    
    static final double kP = 0.025;
    static final double kI = 0.00;
    static final double kD = 0.1;
    static final double kF = 0.00;
    
    static final double kToleranceDegrees = 2.0f;    
    
    static final double kTargetAngleDegrees = -180.0f;
    
    // Channels for the wheels
    final static int LEFT_BACK	= 1;
    final static int RIGHT_BACK	= 5;
    final static int LEFT_FRONT	= 4;
    final static int RIGHT_FRONT = 2;
    
    WPI_TalonSRX leftBack;
    WPI_TalonSRX rightBack;
    WPI_TalonSRX leftFront;
    WPI_TalonSRX rightFront;

    SpeedControllerGroup left;
    SpeedControllerGroup right;

    XboxController controller; 

    public static final int A_ID = 1;
    public static final int B_ID = 2;
    public static final int BACK_ID = 7;

    public Robot() {
    	leftBack = new WPI_TalonSRX(LEFT_BACK);
        rightBack = new WPI_TalonSRX(RIGHT_BACK);
        leftFront = new WPI_TalonSRX(LEFT_FRONT);
        rightFront = new WPI_TalonSRX(RIGHT_FRONT);

        leftBack.setInverted(false);
        rightBack.setInverted(false);
        rightFront.setInverted(true);
        leftFront.setInverted(false);

        leftBack.setNeutralMode(NeutralMode.Brake);
        rightBack.setNeutralMode(NeutralMode.Brake);
        rightFront.setNeutralMode(NeutralMode.Brake);
        leftFront.setNeutralMode(NeutralMode.Brake);

        
        
        left = new SpeedControllerGroup(leftBack, leftFront);
        right = new SpeedControllerGroup(rightBack, rightFront);

        myRobot = new DifferentialDrive(left, right); 
        myRobot.setExpiration(0.1);
        controller = new XboxController(1);

        try {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro:
			 * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
			 * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
            ahrs = new AHRS(SerialPort.Port.kUSB); 
        } catch (RuntimeException ex ) {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
        turnController = new PIDController(kP, kI, kD, kF, ahrs, this);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-0.5, 0.5);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.disable();
        
        /* Add the PID Controller to the Test-mode dashboard, allowing manual  */
        /* tuning of the Turn Controller's P, I and D coefficients.            */
        /* Typically, only the P value needs to be modified.                   */
        LiveWindow.addActuator("DriveSystem", "RotateController", turnController);        
    }
    
    /**
     * Runs the motors with tank steering.
     */
    public void operatorControl() {
        myRobot.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
        	if (controller.getRawButton(A_ID)) {
        		/* While this button is held down, rotate to target angle.  
        		 * Since a Tank drive system cannot move forward simultaneously 
        		 * while rotating, all joystick input is ignored until this
        		 * button is released.
        		 */
        		if (!turnController.isEnabled()) {
        			turnController.setSetpoint(kTargetAngleDegrees);
        			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        			turnController.enable();
        		}
        		double leftStickValue = rotateToAngleRate;
        		double rightStickValue = -rotateToAngleRate;
        		myRobot.tankDrive(leftStickValue,  rightStickValue);
        	} else if ( controller.getRawButton(BACK_ID)) {
        		/* "Zero" the yaw (whatever direction the sensor is 
        		 * pointing now will become the new "Zero" degrees.
        		 */
                if(myRobot.isSafetyEnabled()) myRobot.setSafetyEnabled(false);
                ahrs.zeroYaw();
             
        	} else if ( controller.getRawButton(B_ID)) {
        		/* While this button is held down, the robot is in
        		 * "drive straight" mode.  Whatever direction the robot
        		 * was heading when "drive straight" mode was entered
        		 * will be maintained.  The average speed of both 
        		 * joysticks is the magnitude of motion.
        		 */
        		if(!turnController.isEnabled()) {
        			// Acquire current yaw angle, using this as the target angle.
        			turnController.setSetpoint(ahrs.getYaw());
        			rotateToAngleRate = 0; // This value will be updated in the pidWrite() method.
        			turnController.enable();
                }
                if(myRobot.isSafetyEnabled()) myRobot.setSafetyEnabled(false);
        		double magnitude = (controller.getY(Hand.kLeft) + controller.getY(Hand.kRight)) / 2;
        		double leftStickValue = magnitude + rotateToAngleRate;
        		double rightStickValue = magnitude - rotateToAngleRate;
        		myRobot.tankDrive(leftStickValue,  rightStickValue);
        	} else {
        		/* If the turn controller had been enabled, disable it now. */
        		if(turnController.isEnabled()) {
        			turnController.disable();
        		}
                /* Standard tank drive, no driver assistance. */
                myRobot.setSafetyEnabled(true);
                double left = -controller.getY(Hand.kLeft);
                double right = -controller.getY(Hand.kRight);

                if (Math.abs(left) < 0.2) {
                    left = 0;
                }

                
                if (Math.abs(right) < 0.2) {
                    right = 0;
                }

                myRobot.tankDrive(left, right);
                
                System.out.println(ahrs.getYaw());
        	}
            Timer.delay(0.005);		// wait for a motor update time
        }
    }

	@Override
    /* This function is invoked periodically by the PID Controller, */
    /* based upon navX MXP yaw angle input and PID Coefficients.    */
    public void pidWrite(double output) {
        rotateToAngleRate = output;
    }
}