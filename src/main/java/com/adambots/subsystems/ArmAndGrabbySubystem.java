// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ArmAndGrabbySubystem extends SubsystemBase {

  //private final 
  /** Creates a new ArmAndGrabbySubystem. */

Solenoid leftGrabby;
Solenoid rightGrabby;
TalonFX armLifter;
TalonFX leftArmExtender;
TalonFX rightArmExtender;

  public ArmAndGrabbySubystem(Solenoid leftGrabby, Solenoid rightGrabby, TalonFX armLifter, TalonFX leftArmExtender, TalonFX rightArmExtender) {
    this.leftGrabby = leftGrabby;
    this.rightGrabby = rightGrabby;
    this.armLifter = armLifter;
    this.leftArmExtender = leftArmExtender;
    this.rightArmExtender = rightArmExtender;
  }

  public void openGrabby () {
    leftGrabby.set(true);
    rightGrabby.set(true);
  }
  public void closeGrabby () {
    leftGrabby.set(false);
    rightGrabby.set(false);
  }

  public void raiseArm (int speed) {
    armLifter.set(ControlMode.PercentOutput, speed);
  }

  public void lowerArm (int speed) {
    armLifter.set(ControlMode.PercentOutput, -speed);
  }

  public void extendArm (int speed) {
    rightArmExtender.set(ControlMode.PercentOutput, speed);
    leftArmExtender.set(ControlMode.PercentOutput, -speed);
  }

  public void retractArm(int speed) {
    rightArmExtender.set(ControlMode.PercentOutput, -speed);
    leftArmExtender.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
