// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabbyLifterSubsystem extends SubsystemBase {

  private final TalonFX armLifter;
  private final WPI_CANCoder armLifterEncoder;
  private final PIDController pid;
  private final DigitalInput groundSwitch;
  private final DigitalInput upperSwitch;

  private double maxSpeed = Constants.GrabbyConstants.lifterSpeed;

  private double armLifterSpeed = 0;
  private double targetPosition;

  public GrabbyLifterSubsystem(TalonFX armLifter, WPI_CANCoder armLifterEncoder, DigitalInput groundSwitch, DigitalInput upperSwitch) {
    this.armLifter = armLifter;
    armLifter.setInverted(true);
    this.armLifterEncoder = armLifterEncoder;
    targetPosition = armLifterEncoder.getAbsolutePosition();
    this.pid = new PIDController(Constants.GrabbyConstants.lifterP, Constants.GrabbyConstants.lifterI, Constants.GrabbyConstants.lifterD);

    this.groundSwitch = groundSwitch;
    this.upperSwitch = upperSwitch;
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
    targetPosition = armLifterEncoder.getAbsolutePosition() + increment;
    pid.reset();
  }

  public void manualDown(double increment){
    targetPosition = armLifterEncoder.getAbsolutePosition() - increment;
    pid.reset();
  }

  public void stopLifting(){
    targetPosition = armLifterEncoder.getAbsolutePosition();
    pid.reset();
  }

  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("Lifter Encoder", armLifterEncoder.getAbsolutePosition());

    armLifterSpeed = pid.calculate(armLifterEncoder.getAbsolutePosition(), targetPosition);
    armLifterSpeed = MathUtil.clamp(armLifterSpeed, -maxSpeed, maxSpeed);
    failsafes();
    armLifter.set(ControlMode.PercentOutput, armLifterSpeed);

    SmartDashboard.putNumber("Arm Lifter Speed", armLifterSpeed);
    SmartDashboard.putNumber("Fwd Limit Switch", armLifter.getSensorCollection().isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("Rev Limit Switch", armLifter.getSensorCollection().isRevLimitSwitchClosed());
  }

  private void failsafes() {
    if(armLifterEncoder.getAbsolutePosition() <= 1){
      armLifterSpeed = 0;
    }

    if((armLifterEncoder.getAbsolutePosition() <= Constants.GrabbyConstants.groundState.getArmLiftTarget() || armLifter.getSensorCollection().isRevLimitSwitchClosed() == 1) && armLifterSpeed < 0){
      armLifterSpeed = 0;
    }

    if((armLifterEncoder.getAbsolutePosition() >= Constants.GrabbyConstants.initState.getArmLiftTarget() || armLifter.getSensorCollection().isFwdLimitSwitchClosed() == 1) && armLifterSpeed > 0){
      armLifterSpeed = 0;
    }
  }
}
