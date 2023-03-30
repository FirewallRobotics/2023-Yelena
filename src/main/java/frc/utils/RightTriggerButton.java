// package frc.utils;

// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// import edu.wpi.first.wpilibj2.command.button.Trigger;

// import java.util.function.BooleanSupplier;

// import edu.wpi.first.wpilibj.GenericHID;

// /**
//  * A {@link Button} that gets its state from a {@link GenericHID}.
//  */
// public class RightTriggerButton extends Trigger {
// 	/**
// 	 * Create a gamepad axis for triggering commands as if it were a button.
// 	 *
// 	 * @param joystick     The GenericHID object that has the axis (e.g. Joystick, KinectStick,
// 	 *                     etc)
// 	 * @param axisNumber The axis number (see {@link GenericHID#getRawAxis(int) }
// 	 * 
// 	 * @param threshold The threshold above which the axis shall trigger a command
// 	 * @return 
// 	 */
// 	public void GamepadAxisButton(BooleanSupplier bs) {
// 		super(bs);
// 	}
// }