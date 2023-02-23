// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabbyLifterSubsystem extends SubsystemBase {

  private final TalonFX armLifter;
  private final WPI_CANCoder armLifterEncoder;
  private final PIDController pid;

  private double armLifterSpeed = 0;
  private double targetPosition = Constants.GrabbyConstants.initState.getArmLiftTarget();

  public GrabbyLifterSubsystem(TalonFX armLifter, WPI_CANCoder armLifterEncoder) {
    this.armLifter = armLifter;
    armLifter.setInverted(true);
    this.armLifterEncoder = armLifterEncoder;
    this.pid = new PIDController(Constants.GrabbyConstants.lifterP, Constants.GrabbyConstants.lifterI, Constants.GrabbyConstants.lifterD);
  }

  public void changeTarget(double newTarget){
    targetPosition = newTarget;
    pid.reset();
  }

  public void fullUp(){
    targetPosition = Constants.GrabbyConstants.initState.getArmLiftTarget();
    pid.reset();
  }

  public void fullDown(){
    targetPosition = Constants.GrabbyConstants.groundState.getArmLiftTarget();
    pid.reset();
  }

  public void manualUp(){
    targetPosition = armLifterEncoder.getAbsolutePosition() + 50;
  }

  public void manualDown(){
    targetPosition = armLifterEncoder.getAbsolutePosition() - 50;
  }

  public void stopLifting(){
    targetPosition = armLifterEncoder.getAbsolutePosition();
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Lifter Encoder", armLifterEncoder.getAbsolutePosition());

    armLifterSpeed = pid.calculate(armLifterEncoder.getAbsolutePosition(), targetPosition);
    armLifterSpeed = MathUtil.clamp(armLifterSpeed, -Constants.GrabbyConstants.lifterSpeed, Constants.GrabbyConstants.lifterSpeed);
    failsafes();
    armLifter.set(ControlMode.PercentOutput, armLifterSpeed);

    SmartDashboard.putNumber("Arm Lifter Speed", armLifterSpeed);
  }

  private void failsafes() {
    if(armLifterEncoder.getAbsolutePosition() <= Constants.GrabbyConstants.groundState.getArmLiftTarget() && armLifterSpeed < 0){
      armLifterSpeed = 0;
    }

    if(armLifterEncoder.getAbsolutePosition() >= Constants.GrabbyConstants.initState.getArmLiftTarget() && armLifterSpeed > 0){
      armLifterSpeed = 0;
    }
  }
}