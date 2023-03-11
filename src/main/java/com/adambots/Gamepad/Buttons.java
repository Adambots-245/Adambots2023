/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.adambots.Gamepad;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.adambots.Constants.GamepadConstants;
import com.adambots.RobotMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * All Game Controller Button Mappings
 */
public class Buttons {

        // initialize controllers
        public static final CommandXboxController primaryJoystick = new CommandXboxController(
                        GamepadConstants.kPrimaryDriver);
        public static final CommandXboxController secondaryJoystick = new CommandXboxController(
                        GamepadConstants.kSecondaryDriver);
        public static final CommandJoystick ex3dPro = new CommandJoystick(RobotMap.kJoystickControllerPort);
        public static final BooleanSupplier isJoystickConnected = () -> ex3dPro.getHID().isConnected()
                        || RobotBase.isSimulation();

        // primary buttons
        public static final Trigger primaryBackButton = primaryJoystick.back();
        public static final Trigger primaryStartButton = primaryJoystick.start();
        public static final Trigger primaryXButton = primaryJoystick.x();
        public static final Trigger primaryYButton = primaryJoystick.y();
        public static final Trigger primaryBButton = primaryJoystick.b();
        public static final Trigger primaryAButton = primaryJoystick.a();
        public static final Trigger primaryLB = primaryJoystick.leftBumper();
        public static final Trigger primaryRB = primaryJoystick.rightBumper();
        public static final Trigger primaryLeftStickButton = primaryJoystick.leftStick();
        public static final Trigger primaryRightStickButton = primaryJoystick.rightStick();

        // primary DPad
        public static final Trigger primaryDPadN = primaryJoystick.pov(GamepadConstants.kDpadNAngle);
        public static final Trigger primaryDPadNW = primaryJoystick.pov(GamepadConstants.kDpadNWAngle);
        public static final Trigger primaryDPadW = primaryJoystick.pov(GamepadConstants.kDpadWAngle);
        public static final Trigger primaryDPadSW = primaryJoystick.pov(GamepadConstants.kDpadSWAngle);
        public static final Trigger primaryDPadS = primaryJoystick.pov(GamepadConstants.kDpadSAngle);
        public static final Trigger primaryDPadSE = primaryJoystick.pov(GamepadConstants.kDpadSEAngle);
        public static final Trigger primaryDPadE = primaryJoystick.pov(GamepadConstants.kDpadEAngle);
        public static final Trigger primaryDPadNE = primaryJoystick.pov(GamepadConstants.kDpadNEAngle);

        // primary axes
        // RIGHT TRIGGER primaryJoystick.getRightTriggerAxis()
        // LEFT TRIGGER primaryJoystick.getLeftTriggerAxis()
        // LEFT STICK X AXIS primaryJoystick.getLeftX()
        // LEFT STICK Y AXIS primaryJoystick.getLeftY()
        // RIGHT STICK X AXIS primaryJoystick.getRightX()
        // RIGHT STICK Y AXIS primaryJoystick.getRightY()

        // secondary buttons
        public static final Trigger secondaryBackButton = secondaryJoystick.back();
        public static final Trigger secondaryStartButton = secondaryJoystick.start();
        public static final Trigger secondaryXButton = secondaryJoystick.x();
        public static final Trigger secondaryYButton = secondaryJoystick.y();
        public static final Trigger secondaryBButton = secondaryJoystick.b();
        public static final Trigger secondaryAButton = secondaryJoystick.a();
        public static final Trigger secondaryLB = secondaryJoystick.leftBumper();
        public static final Trigger secondaryRB = secondaryJoystick.rightBumper();
        public static final Trigger secondaryLeftStickButton = secondaryJoystick.leftStick();
        public static final Trigger secondaryRightStickButton = secondaryJoystick.rightStick();

        // secondary DPad
        public static final Trigger secondaryDPadN = secondaryJoystick.pov(GamepadConstants.kDpadNAngle);
        public static final Trigger secondaryDPadNW = secondaryJoystick.pov(GamepadConstants.kDpadNWAngle);
        public static final Trigger secondaryDPadW = secondaryJoystick.pov(GamepadConstants.kDpadWAngle);
        public static final Trigger secondaryDPadSW = secondaryJoystick.pov(GamepadConstants.kDpadSWAngle);
        public static final Trigger secondaryDPadS = secondaryJoystick.pov(GamepadConstants.kDpadSAngle);
        public static final Trigger secondaryDPadSE = secondaryJoystick.pov(GamepadConstants.kDpadSEAngle);
        public static final Trigger secondaryDPadE = secondaryJoystick.pov(GamepadConstants.kDpadEAngle);
        public static final Trigger secondaryDPadNE = secondaryJoystick.pov(GamepadConstants.kDpadNEAngle);

        // secondary axes
        // RIGHT TRIGGER secondaryJoystick.getRightTriggerAxis()
        // LEFT TRIGGER secondaryJoystick.getLeftTriggerAxis()
        // LEFT STICK X AXIS secondaryJoystick.getLeftX()
        // LEFT STICK Y AXIS secondaryJoystick.getLeftY()
        // RIGHT STICK X AXIS secondaryJoystick.getRightX()
        // RIGHT STICK Y AXIS secondaryJoystick.getRightY()

        public static final Trigger JoystickButton1 = ex3dPro.button(1);
        public static final Trigger JoystickButton2 = ex3dPro.button(2);
        public static final Trigger JoystickButton3 = ex3dPro.button(3);
        public static final Trigger JoystickButton4 = ex3dPro.button(4);
        public static final Trigger JoystickButton5 = ex3dPro.button(5);
        public static final Trigger JoystickButton6 = ex3dPro.button(6);
        public static final Trigger JoystickButton7 = ex3dPro.button(7);
        public static final Trigger JoystickButton8 = ex3dPro.button(8);
        public static final Trigger JoystickButton9 = ex3dPro.button(9);
        public static final Trigger JoystickButton10 = ex3dPro.button(10);
        public static final Trigger JoystickButton11 = ex3dPro.button(11);
        public static final Trigger JoystickButton12 = ex3dPro.button(12);

        public static final Trigger JoystickThumbUp = ex3dPro.povUp();
        public static final Trigger JoystickThumbDown = ex3dPro.povDown();
        public static final Trigger JoystickThumbUpLeft = ex3dPro.povUpLeft();
        public static final Trigger JoystickThumbUpRight = ex3dPro.povUpRight();
        public static final Trigger JoystickThumbDownLeft = ex3dPro.povDownLeft();
        public static final Trigger JoystickThumbDownRight = ex3dPro.povDownRight();
        public static final Trigger JoystickThumbLeft = ex3dPro.povLeft();
        public static final Trigger JoystickThumbRight = ex3dPro.povRight();
        public static final Trigger JoystickThumbCenter = ex3dPro.povCenter();

        // deadzoning
        public static double deaden(double rawInput) {
                return Math.abs(rawInput) < GamepadConstants.kDeadZone ? 0 : rawInput;
        }

        public static double deaden(double input, double sadDeadenVariable) {
                if (Math.abs(input) < sadDeadenVariable) {
                        return 0;
                } else {
                        return input;
                }
        }

        //Applies a custom curve to an input and returns the result
        public static double applyCurve (double rawInput, double[] curve) {
                double scaled = Math.abs(rawInput)*10;
                int index = (int)Math.floor(scaled);
                return MathUtil.interpolate(curve[index], curve[Math.min(index+1, curve.length-1)], scaled-index)*Math.signum(rawInput);
	}

        // An example curve that would be default input {0, 0.1, 0.2, 0.3, 0.4, 0.5,
        // 0.6, 0.7, 0.8, 0.9, 1};
        static double[] forwardCurve = { 0, 0, 0.15, 0.2125, 0.325, 0.4375, 0.55, 0.6625, 0.775, 0.8875, 1 }; //Scaled back linear curve
        static double[] sidewaysCurve = { 0, 0.0, 0.0, 0, 0.1, 0.2, 0.55, 0.6625, 0.775, 0.8875, 1 };
        static double[] rotateCurve = { 0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.4, 0.6, 0.8, 1 }; //first 4 left empty for deadzone

        // Sigmoid Curve
        public static double smoothInput(double input) {
                // Adjust the parameter 'a' to control the steepness of the curve
                double a = 4.0;

                // Apply a sigmoid function to the input value
                double sigmoid = 1.0 / (1.0 + Math.exp(-a * input));

                // Apply a hyperbolic tangent function to the input value
                // double tanh = Math.tanh(a * input);

                // Map the output range from (0,1) to (minOutput, maxOutput)
                double minOutput = -1; // minimum output value
                double maxOutput = 1; // maximum output value
                double output = sigmoid * (maxOutput - minOutput) + minOutput;
                // double output = tanh * (maxOutput - minOutput) + minOutput;

                return output;
        }

        public static double cubic(double input) {
                double tuneA = 0; // try different values. However, tuneA and tuneB should add up to 1
                double tuneB = 1;

                return (tuneA * input) + (tuneB * Math.pow(input, 3));
        }

        // If Flight Joystick is connected, then return Joystick Y value - else return
        // Joystick value from XBoxController
        /**
         * getY or LeftY
         */
        public static DoubleSupplier forwardSupplier = () -> isJoystickConnected.getAsBoolean()
                        // ? deaden(ex3dPro.getY(), GamepadConstants.kDeadZone)
                        ? applyCurve(ex3dPro.getY(), forwardCurve) // CHANGE BACK IF IT DOESNT WORK
                        // ? deaden(smoothInput(ex3dPro.getY()), GamepadConstants.kDeadZone)
                        : deaden(primaryJoystick.getLeftY(), GamepadConstants.kDeadZone);

        /**
         * getX or LeftX
         */
        public static DoubleSupplier sidewaysSupplier = () -> isJoystickConnected.getAsBoolean()
                        // ? deaden(ex3dPro.getX(), GamepadConstants.kDeadZone)
                        ? applyCurve(ex3dPro.getX(), sidewaysCurve) // CHANGE BACK IF IT DOESNT WORK
                        // ? deaden(smoothInput(ex3dPro.getX()), GamepadConstants.kDeadZone)
                        : deaden(primaryJoystick.getLeftX(), GamepadConstants.kDeadZone);

        /**
         * getZ or RightX
         */
        public static DoubleSupplier rotateSupplier = () -> isJoystickConnected.getAsBoolean()
                        // ? deaden(ex3dPro.getZ(), 0.4)
                        ? applyCurve(ex3dPro.getZ(), rotateCurve) // CHANGE BACK IF IT DOESNT WORK
                        // ? deaden(smoothInput(ex3dPro.getZ()), 0.4)
                        : deaden(primaryJoystick.getRightX(), GamepadConstants.kDeadZone);

        /** Rumble the XBox Controller 
         * @param controller pass the primary or secondary controller to rumble
         * @param timeInMillis how many milliseconds to rumble the controller - max value is 5000
         * @param intensity0to1 how intense should the rumble be
         * 
         * Example: Buttons.rumble(Buttons.primaryJoystick, 2000, 1)
        */
        public static void rumble(CommandXboxController controller, int timeInMillis, int intensity0to1){

                var joy = controller.getHID();
                final int time = MathUtil.clamp(timeInMillis, 0, 5000);

                // Perform an async operation to avoid scheduler overruns
                Thread rumbleThread = new Thread(() -> {

                        long rumbleStartTime = System.currentTimeMillis();
                        
                        while (System.currentTimeMillis() - rumbleStartTime <= time) {
                                joy.setRumble(RumbleType.kBothRumble, intensity0to1); // Rumble both sides of the controller
                        }
                        
                        joy.setRumble(RumbleType.kBothRumble, 0);
                });

                rumbleThread.start();
        }
}
