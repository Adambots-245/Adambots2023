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

private final DoubleSolenoid grabby;
// Solenoid rightGrabby;
private final TalonFX armLifter;
private final TalonFX leftArmExtender;
private final TalonFX rightArmExtender;

private double lifterSpeed = Constants.GrabbyConstants.lifterSpeed;
private double extenderSpeed = Constants.GrabbyConstants.extenderSpeed;

private final double lowZoneLifterValue = Constants.GrabbyConstants.lowZoneLifterValue;
private final double lowZoneExtenderValue = Constants.GrabbyConstants.lowZoneExtenderValue;


private final int armEncoderCPR = Constants.GrabbyConstants.armEncoderCPR;

  public ArmAndGrabbySubystem(DoubleSolenoid grabby, /*Solenoid rightGrabby,*/ TalonFX armLifter, TalonFX leftArmExtender, TalonFX rightArmExtender) {
    this.grabby = grabby;
   // this.rightGrabby = rightGrabby;
    this.armLifter = armLifter;
    this.leftArmExtender = leftArmExtender;
    this.rightArmExtender = rightArmExtender;
    
    armLifter.setSelectedSensorPosition(0);
    leftArmExtender.setSelectedSensorPosition(0);
    rightArmExtender.setSelectedSensorPosition(0);  
  }

  public void openGrabby() {
    grabby.set(Value.kForward);
    // rightGrabby.set(true);
  }

  public void closeGrabby() {
    grabby.set(Value.kReverse);
    // rightGrabby.set(false);
  }

  public void raiseArm() {
    armLifter.set(ControlMode.PercentOutput, lifterSpeed);
  }

  public void lowerArm() {
    armLifter.set(ControlMode.PercentOutput, -lifterSpeed);
  }

  public void stopArmLifter() {
    armLifter.set(ControlMode.PercentOutput, 0);
  }

  public void extendArm() {
    leftArmExtender.set(ControlMode.PercentOutput, -extenderSpeed);
    rightArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
  }

  public void retractArm() {
    leftArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
    rightArmExtender.set(ControlMode.PercentOutput, -extenderSpeed);
  }

  public void stopArmExtender() {
    leftArmExtender.set(ControlMode.PercentOutput, 0);
    rightArmExtender.set(ControlMode.PercentOutput, 0);
  }

  public double getArmLifterEncoder() {
    return armLifter.getSelectedSensorPosition() / armEncoderCPR;
  }

  public double getLeftExtenderEncoder() {
    return leftArmExtender.getSelectedSensorPosition() / armEncoderCPR;
  }

  public double getRightExtenderEncoder() {
    return rightArmExtender.getSelectedSensorPosition() / armEncoderCPR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("armEncoder", getArmLifterEncoder());
    SmartDashboard.putNumber("leftExtenderEncoder", getRightExtenderEncoder());
    SmartDashboard.putNumber("rightExtenderEncoder", getLeftExtenderEncoder());

    if (Math.abs(lowZoneLifterValue - getArmLifterEncoder()) > 0.1) {
      if (getArmLifterEncoder() > lowZoneLifterValue) {
        lowerArm();
      } else if (getArmLifterEncoder() < lowZoneLifterValue) {
          raiseArm();
      } 
    } else {
        stopArmExtender();
    }

    armLifter.set(ControlMode.PercentOutput, lifterSpeed);
    leftArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
    leftArmExtender.set(ControlMode.PercentOutput, extenderSpeed);

     /*
      if (getRightExtenderEncoder() > Constants.GrabbyConstants.lowZoneExtenderValue) {
        if (Math.abs(Constants.GrabbyConstants.lowZoneLifterValue - getArmLifterEncoder()) > 1) {
           lowerArm(0.1);
         } else if (getArmLifterEncoder() > Constants.GrabbyConstants.lowZoneLifterValue) {
           raiseArm(0.1);
         }
      */

  }

}
