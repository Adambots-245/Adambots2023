// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

/**
 * Convert a motor connected to a Spark Max Controller to be a stepper motor.
 * The motor will have soft limits to prevent rotation beyond a specific angle
 * and also,
 * will step up and down in small increments.
 * 
 * Usage:
 * motor = new StepperMotor(RobotMap.grabbyMotor, 90, false);
 * 
 * motor.stepUp(20); // Step up by 20 degrees
 * motor.stepDown(20); // Step down by 20 degrees
 */
public class StepperMotor {

    private CANSparkMax motorController;
    private float maxAngle;
    private SparkMaxPIDController pidController;
    private final double kP = 1.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kFF = 0.0;

    /**
     * Create a new Stepper Motor. Limit rotation to maxAngle.
     * @param motorController a CANSparkMax controller to use
     * @param maxAngle Maximum angle to limit the motor. Motor will be allowed to go + or - this angle
     * @param isInverted Should the direction of the motor be reversed?
     */
    public StepperMotor(CANSparkMax motorController, float maxAngle, boolean isInverted) {
        this.motorController = motorController;
        maxAngle = maxAngle / 360.0f;
        this.maxAngle = maxAngle;

        this.motorController.clearFaults();
        this.motorController.setInverted(isInverted);

        pidController = motorController.getPIDController();
        motorController.getEncoder().setPositionConversionFactor(1.0);

        motorController.setSoftLimit(SoftLimitDirection.kForward, this.maxAngle);
        motorController.setSoftLimit(SoftLimitDirection.kReverse, -this.maxAngle);

        motorController.enableSoftLimit(SoftLimitDirection.kForward, true);
        motorController.enableSoftLimit(SoftLimitDirection.kReverse, true);

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setFF(kFF);

        // Set the position reference for the encoder
        motorController.getEncoder().setPosition(0);
    }

    /**
     * Step up by increment angle
     * @param increment in positive degrees
     */
    public void stepUp(double increment) {
        double currentPos = motorController.getEncoder().getPosition();

        pidController.setReference(currentPos + (increment/360.0), ControlType.kPosition);
    }

    /** Step down by decrement angle
     * @param decrement in positive degrees
     */
    public void stepDown(double decrement) {
        double currentPos = motorController.getEncoder().getPosition();

        pidController.setReference(currentPos - (decrement/360.0), ControlType.kPosition);
    }
}
