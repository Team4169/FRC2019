/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class RobotMap {
    // motor controllers
	public static final int
			// drive train
			LEFT_FRONT = 4, //pdp 0
			RIGHT_FRONT = 1, //pdp 1
			LEFT_BACK = 3, //pdp 2
			RIGHT_BACK = 2, //pdp 3

			PDP_LF = 0,
			PDP_RF = 1,
			PDP_LB = 2,
			PDP_RB = 3,
			
			// hatch motors
			ARMMOTOR = 7,
			EXTENSION = 0,

      		// hatch limit switch
    		LIMIT_SWITCH_PORT = 0,

			//climber motors
			CLIMBER1 = 5,
			CLIMBER2 = 6,

			// Xbox Controller
			CONTROLLER_PORT1 = 0,
			CONTROLLER_PORT2 = 1,
			A_ID = 1,
			B_ID = 2,
			X_ID = 3,
			Y_ID = 4,
			LB_ID = 5,
			RB_ID = 6,
			BACK_ID = 7,
			START_ID = 8,
			LEFT_JOY_ID = 9,
			RIGHT_JOY_ID = 10;
}
