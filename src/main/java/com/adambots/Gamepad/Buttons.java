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
    public static final BooleanSupplier isJoystickConnected = () -> ex3dPro.getHID().isConnected();

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

    // If Flight Joystick is connected, then return Joystick Y value - else return
    // Joystick value from XBoxController
    /**
     * getY or LeftY
     */
    public static double forward = isJoystickConnected.getAsBoolean() ? deaden(ex3dPro.getY(), GamepadConstants.kDeadZone)
            : deaden(primaryJoystick.getLeftY(), GamepadConstants.kDeadZone);
    /**
     * getX or LeftX
     */
    public static double sideways = isJoystickConnected.getAsBoolean() ? deaden(ex3dPro.getX(), GamepadConstants.kDeadZone)
            : deaden(primaryJoystick.getLeftX(), 0.3);
    /**
     * getZ or RightX
     */
    public static double rotate = isJoystickConnected.getAsBoolean() ? deaden(ex3dPro.getZ(), 0.314)
            : deaden(primaryJoystick.getRightX(), 0.25);

    public static DoubleSupplier forwardSupplier = () -> isJoystickConnected.getAsBoolean()
            ? deaden(ex3dPro.getY(), GamepadConstants.kDeadZone)
            : deaden(primaryJoystick.getLeftY(), GamepadConstants.kDeadZone);
    public static DoubleSupplier sidewaysSupplier = () -> isJoystickConnected.getAsBoolean()
            ? deaden(ex3dPro.getX(), GamepadConstants.kDeadZone)
            : deaden(primaryJoystick.getLeftX(), 0.3);
    public static DoubleSupplier rotateSupplier = () -> isJoystickConnected.getAsBoolean()
            ? deaden(ex3dPro.getZ(), 0.314)
            : deaden(primaryJoystick.getRightX(), 0.25);
}
