// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.utils.Dash;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabbyLifterSubsystem extends SubsystemBase {

  private final TalonFX armLifter;
  private final CANcoder armLifterEncoder;
  private final PIDController pid;

  private double maxSpeed = Constants.GrabbyConstants.lifterSpeed;

  private double armLifterSpeed = 0;
  private double targetPosition;

  public GrabbyLifterSubsystem(TalonFX armLifter, CANcoder armLifterEncoder) {
    this.armLifter = armLifter;
    armLifter.setInverted(true);

    this.armLifterEncoder = armLifterEncoder;
    targetPosition = armLifterEncoder.getAbsolutePosition().getValueAsDouble();

    this.pid = new PIDController(Constants.GrabbyConstants.lifterP, Constants.GrabbyConstants.lifterI, Constants.GrabbyConstants.lifterD);

    Dash.add("Lifter Encoder", () -> armLifterEncoder.getAbsolutePosition().getValueAsDouble());
    Dash.add("Arm Lifter Speed", () -> armLifterSpeed);
    // Dash.add("Fwd Limit Switch", () -> armLifter.getSensorCollection().isFwdLimitSwitchClosed());
    // Dash.add("Rev Limit Switch", () -> armLifter.getSensorCollection().isRevLimitSwitchClosed());
    Dash.add("Fwd Limit Switch", () -> armLifter.getForwardLimit().getValueAsDouble());
    Dash.add("Rev Limit Switch", () -> armLifter.getReverseLimit().getValueAsDouble());
  }

  public void changeTarget(double newTarget){
    targetPosition = newTarget;
    pid.reset();
  }

  public double getState() {
    return targetPosition;
  }

  public void fullUp(){
    targetPosition = Constants.GrabbyConstants.initState.getArmLiftTarget();
    pid.reset();
  }

  public void fullDown(){
    targetPosition = Constants.GrabbyConstants.groundState.getArmLiftTarget();
    pid.reset();
  }

  public void changeMaxSpeed(double newMax){
    maxSpeed = newMax;
  }

  public void manualUp(double increment){
    targetPosition = getEncoder() + increment;
    pid.reset();
  }

  public void manualDown(double increment){
    targetPosition = getEncoder() - increment;
    pid.reset();
  }

  public void stopLifting(){
    targetPosition = getEncoder();
    pid.reset();
  }

  public double getEncoder(){
    return armLifterEncoder.getAbsolutePosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    armLifterSpeed = pid.calculate(armLifterEncoder.getAbsolutePosition().getValueAsDouble(), targetPosition);
    armLifterSpeed = MathUtil.clamp(armLifterSpeed, -maxSpeed, maxSpeed);
    failsafes();
    armLifter.set(armLifterSpeed);
  }

  private void failsafes() {
    if(getEncoder() <= 1){
      armLifterSpeed = 0;
    }

    if((getEncoder() <= Constants.GrabbyConstants.groundState.getArmLiftTarget() || armLifter.getReverseLimit().getValueAsDouble() == 1) && armLifterSpeed < 0){
      armLifterSpeed = 0;
    }

    if((getEncoder() >= Constants.GrabbyConstants.initState.getArmLiftTarget() || armLifter.getForwardLimit().getValueAsDouble() == 1) && armLifterSpeed > 0){
      armLifterSpeed = 0;
    }
  }
}
