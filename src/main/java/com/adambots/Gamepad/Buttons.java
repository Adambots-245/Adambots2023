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

        // primary analog inputs
        public static Trigger leftTrigger = new Trigger(()-> {
                return primaryJoystick.getLeftTriggerAxis() > 0.5;
        });
        public static Trigger rightTrigger = new Trigger(()-> {
                return primaryJoystick.getRightTriggerAxis() > 0.5;
        });

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
        public static final Trigger JoystickButton13 = ex3dPro.button(13);
        public static final Trigger JoystickButton14 = ex3dPro.button(14);
        public static final Trigger JoystickButton15 = ex3dPro.button(15);
        public static final Trigger JoystickButton16 = ex3dPro.button(16);

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
        public static double deaden(double input, double deadenThreshold) {
                if (Math.abs(input) < deadenThreshold) {
                        return 0;
                } else {
                        return input;
                }
        }

        //Applies a custom curve to an input and returns the result
        public static double applyCurve (double rawInput, double[] curve) {
                return curve[(int)Math.floor(Math.abs(rawInput)*100)]*Math.signum(rawInput);          
	}

        // DO NOT ADJUST THESE MANUALLY!!!
        // Go to Adambots-245/Utils/Curve_Creator and you can import these curves into the program to adjust them, or make new ones
        // The 2D arrays function like points on a graph (x, y)
        // an example curve that would be default linear input from (0,0) to (1,1) is {{0.0, 0.0},{1.0, 1.0}}
        static double[] forwardCurve = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.005,0.01,0.015,0.02,0.025,0.03,0.035,0.04,0.045,0.05,0.055,0.06,0.065,0.07,0.075,0.08,0.085,0.09,0.095,0.1,0.105,0.11,0.115,0.12,0.125,0.13,0.135,0.14,0.145,0.15,0.155,0.16,0.165,0.17,0.175,0.18,0.185,0.19,0.195,0.2,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4,0.42,0.44,0.46,0.48,0.5,0.52,0.54,0.56,0.58,0.6,0.62,0.64,0.66,0.68,0.7,0.72,0.74,0.76,0.78,0.8,0.82,0.84,0.86,0.88,0.9,0.92,0.94,0.96,0.98,1.0};
        static double[] sidewaysCurve = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.005,0.01,0.015,0.02,0.025,0.03,0.035,0.04,0.045,0.05,0.055,0.06,0.065,0.07,0.075,0.08,0.085,0.09,0.095,0.1,0.105,0.11,0.115,0.12,0.125,0.13,0.135,0.14,0.145,0.15,0.155,0.16,0.165,0.17,0.175,0.18,0.185,0.19,0.195,0.2,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4,0.42,0.44,0.46,0.48,0.5,0.52,0.54,0.56,0.58,0.6,0.62,0.64,0.66,0.68,0.7,0.72,0.74,0.76,0.78,0.8,0.82,0.84,0.86,0.88,0.9,0.92,0.94,0.96,0.98,1.0};
        static double[] rotateCurve = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.007,0.014,0.021,0.028,0.035,0.042,0.049,0.056,0.063,0.07,0.0765,0.083,0.0895,0.096,0.1025,0.109,0.1155,0.122,0.1285,0.135,0.1415,0.148,0.1545,0.161,0.1675,0.174,0.1805,0.187,0.1935,0.2,0.21,0.22,0.23,0.24,0.25,0.26,0.27,0.28,0.29,0.3,0.31,0.32,0.33,0.34,0.35,0.36,0.37,0.38,0.39,0.4,0.42,0.44,0.46,0.48,0.5,0.52,0.54,0.56,0.58,0.6,0.62,0.64,0.66,0.68,0.7,0.72,0.74,0.76,0.78,0.8,0.82,0.84,0.86,0.88,0.9,0.92,0.94,0.96,0.98,1.0};
        // static double[][] forwardCurve = {{0.0, 0.0},{0.1, 0.0},{0.2, 0.05},{0.5, 0.2},{0.7, 0.4},{0.8, 0.6},{1.0, 1.0}};
        // static double[][] sidewaysCurve = {{0.0, 0.0},{0.1, 0.0},{0.2, 0.05},{0.5, 0.2},{0.7, 0.4},{0.8, 0.6},{1.0, 1.0}};
        // static double[][] rotateCurve = {{0.0, 0.0},{0.2, 0.0},{0.3, 0.07},{0.5, 0.2},{0.7, 0.4},{1.0, 1.0}};


        // If Flight Joystick is connected, then return Joystick Y value - else return
        // Joystick value from XBoxController
        /**
         * getY or LeftY
         */
        public static DoubleSupplier forwardSupplier = () -> isJoystickConnected.getAsBoolean()
                        ? applyCurve(ex3dPro.getY(), forwardCurve)
                        : deaden(primaryJoystick.getLeftY(), GamepadConstants.kDeadZone);

        /**
         * getX or LeftX
         */
        public static DoubleSupplier sidewaysSupplier = () -> isJoystickConnected.getAsBoolean()
                        ? applyCurve(ex3dPro.getX(), sidewaysCurve)
                        : deaden(primaryJoystick.getLeftX(), GamepadConstants.kDeadZone);

        /**
         * getZ or RightX
         */
        public static DoubleSupplier rotateSupplier = () -> isJoystickConnected.getAsBoolean()
                        ? applyCurve(ex3dPro.getZ(), rotateCurve)
                        : deaden(primaryJoystick.getRightX(), GamepadConstants.kDeadZone);
        public static DoubleSupplier rotateSupplier2 = () -> ex3dPro.getZ();

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
