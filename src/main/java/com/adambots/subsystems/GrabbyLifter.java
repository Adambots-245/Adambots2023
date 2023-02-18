// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabbyLifter extends SubsystemBase {

  private final TalonFX armLifter;
  private final WPI_CANCoder armLifterEncoder;
  private final PIDController pid;

  private double armLifterSpeed = 0;
  private double targetPosition = Constants.GrabbyConstants.initState.getArmLiftTarget();

  public GrabbyLifter(TalonFX armLifter, WPI_CANCoder armLifterEncoder) {
    this.armLifter = armLifter;
    this.armLifterEncoder = armLifterEncoder;
    this.pid = new PIDController(0, 0, 0);
  }

  public void changeTarget(int newTarget){
    targetPosition = newTarget;
  }

  public void armFullUp(){
    targetPosition = Constants.GrabbyConstants.initState.getArmLiftTarget();
  }

  public void armFullDown(){
    targetPosition = Constants.GrabbyConstants.groundState.getArmLiftTarget();
  }

  @Override
  public void periodic() {
    
    failsafes();
    armLifter.set(ControlMode.PercentOutput, armLifterSpeed);

  }

  private void failsafes() {
    if(armLifterEncoder.getAbsolutePosition() <= Constants.GrabbyConstants.groundState.getArmLiftTarget() && armLifterSpeed > 0){
      armLifterSpeed = 0;
    }

    if(armLifterEncoder.getAbsolutePosition() >= Constants.GrabbyConstants.initState.getArmLiftTarget() && armLifterSpeed < 0){
      armLifterSpeed = 0;
    }
  }
}
