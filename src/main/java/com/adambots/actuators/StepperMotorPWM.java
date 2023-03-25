// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.actuators;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

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
public class StepperMotorPWM {

    private PWMSparkMax motorController;
    private float maxAngle;
    private SparkMaxPIDController pidController;
    private final double kP = 1.0;
    private final double kI = 0.0;
    private final double kD = 0.0;
    private final double kFF = 0.0;

    /**
     * Create a new Stepper Motor. Limit rotation to maxAngle.
     * @param motorController a PWMSparkMax controller to use
     * @param maxAngle Maximum angle to limit the motor. Motor will be allowed to go + or - this angle
     * @param isInverted Should the direction of the motor be reversed?
     */
    public StepperMotorPWM(PWMSparkMax motorController, float maxAngle, boolean isInverted) {
        this.motorController = motorController;
        maxAngle = maxAngle / 360.0f;
        this.maxAngle = maxAngle;

        this.motorController.setInverted(isInverted);
    }

    public void setSpeed(double speed){
        motorController.set(speed);
    }

    public void stop(){
        motorController.set(0);
    }
}
