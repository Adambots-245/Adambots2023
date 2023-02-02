// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.RobotMap;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.adambots.commands.*;
import com.adambots.Constants;



public class ArmAndGrabbySubystem extends SubsystemBase {

  //private final 
  /** Creates a new ArmAndGrabbySubystem. */

DoubleSolenoid grabby;
// Solenoid rightGrabby;
TalonFX armLifter;
TalonFX leftArmExtender;
TalonFX rightArmExtender;

  public ArmAndGrabbySubystem(DoubleSolenoid grabby, /*Solenoid rightGrabby,*/ TalonFX armLifter, TalonFX leftArmExtender, TalonFX rightArmExtender) {
    this.grabby = grabby;
   // this.rightGrabby = rightGrabby;
    this.armLifter = armLifter;
    this.leftArmExtender = leftArmExtender;
    this.rightArmExtender = rightArmExtender;
  }

  public void openGrabby () {
    grabby.set(Value.kForward);
    // rightGrabby.set(true);
  }
  public void closeGrabby () {
    grabby.set(Value.kReverse);
    // rightGrabby.set(false);
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

  public void ScoreInLowZone (int speed) {
    // if (armLifter.getSelectedSensorPosition()
    // if (getArmLifterEncoder() < position) {
    //   armLifter.set(ControlMode.PercentOutput, speed);
    // }
    // else {
    //   armLifter.set(ControlMode.PercentOutput, 0);
    // }
  }

  public double getArmLifterEncoder () {
    return armLifter.getSelectedSensorPosition() / Constants.ModuleConstants.kEncoderCPR;
  }

  public double getLeftExtenderEncoder () {
    return leftArmExtender.getSelectedSensorPosition() / Constants.ModuleConstants.kEncoderCPR;
  }

  public double getRightExtenderEncoder () {
    return rightArmExtender.getSelectedSensorPosition() / Constants.ModuleConstants.kEncoderCPR  ;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("armEncoder", getArmLifterEncoder());
    SmartDashboard.putNumber("leftExtenderEncoder", getArmLifterEncoder());
    SmartDashboard.putNumber("rightExtenderEncoder", getArmLifterEncoder());


    // This method will be called once per scheduler run
  }
}
