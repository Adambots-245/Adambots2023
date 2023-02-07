// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmAndGrabbySubystem extends SubsystemBase {

  //private final 
  /** Creates a new ArmAndGrabbySubystem. */

private final DoubleSolenoid grabby;
// Solenoid rightGrabby;
private final TalonFX armLifter;
private final TalonFX leftArmExtender;
private final TalonFX rightArmExtender;
private final DigitalInput rightExtenderLimit;
private final DigitalInput leftExtenderLimit;

private double lifterSpeed = Constants.GrabbyConstants.lifterSpeed;
private double extenderSpeed = Constants.GrabbyConstants.extenderSpeed;

private boolean manualControl = false;
private double lifterSpeedCurrent = 0;
private double rightExtenderSpeedCurrent = 0;
private double leftExtenderSpeedCurrent = 0;

private double extendTarget = Constants.GrabbyConstants.lowZoneExtenderValue;
private double liftTarget = Constants.GrabbyConstants.lowZoneLifterValue;
private double extendInit;
private double liftInit;

private final int armEncoderCPR = Constants.GrabbyConstants.armEncoderCPR;

  public ArmAndGrabbySubystem(DoubleSolenoid grabby, /*Solenoid rightGrabby,*/ TalonFX armLifter, TalonFX leftArmExtender, TalonFX rightArmExtender, DigitalInput rightExtenderLimit, DigitalInput leftExtenderLimit) {
    this.grabby = grabby;
   // this.rightGrabby = rightGrabby;
    this.armLifter = armLifter;
    this.leftArmExtender = leftArmExtender;
    this.rightArmExtender = rightArmExtender;
    this.leftExtenderLimit = leftExtenderLimit;
    this.rightExtenderLimit = rightExtenderLimit;
    
    armLifter.setSelectedSensorPosition(0);
    leftArmExtender.setSelectedSensorPosition(0);
    rightArmExtender.setSelectedSensorPosition(0);  
    extendInit = getRightExtenderEncoder();
    liftInit = getArmLifterEncoder();
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
    manualControl = true;
    lifterSpeedCurrent = lifterSpeed;
  }

  public void lowerArm() {
    manualControl = true;
    lifterSpeedCurrent = -lifterSpeed;
  }

  public void stopArmLifter() {
    lifterSpeedCurrent = 0;
  }

  public void extendArm() {
    manualControl = true;
    rightExtenderSpeedCurrent = extenderSpeed;
    leftExtenderSpeedCurrent = extenderSpeed;

    //leftArmExtender.set(ControlMode.PercentOutput, -extenderSpeed);
    //rightArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
  }

  public void retractArm() {
    manualControl = true;
    rightExtenderSpeedCurrent = -extenderSpeed;
    leftExtenderSpeedCurrent = -extenderSpeed;
    //leftArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
    //rightArmExtender.set(ControlMode.PercentOutput, -extenderSpeed);
  }

  public void stopArmExtender() {
    rightExtenderSpeedCurrent = 0;
    leftExtenderSpeedCurrent = 0;
    //leftArmExtender.set(ControlMode.PercentOutput, 0);
    //rightArmExtender.set(ControlMode.PercentOutput, 0);
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

  public boolean getRightLimit(){
    return rightExtenderLimit.get();
  }
  
  public boolean getLeftLimit(){
    return leftExtenderLimit.get();
  }

  public void lowExtendAndLift() {
    extendInit = getRightExtenderEncoder();
    liftInit = getArmLifterEncoder();
    manualControl = false;
    extendTarget = Constants.GrabbyConstants.lowZoneExtenderValue;
    liftTarget = Constants.GrabbyConstants.lowZoneLifterValue;
  }

  public void midExtendAndLift() {
    extendInit = getRightExtenderEncoder();
    liftInit = getArmLifterEncoder();
    manualControl = false;
    extendTarget = Constants.GrabbyConstants.midZoneExtenderValue;
    liftTarget = Constants.GrabbyConstants.midZoneLifterValue;
  }

  public void highExtendAndLift() {
    extendInit = getRightExtenderEncoder();
    liftInit = getArmLifterEncoder();
    manualControl = false;
    extendTarget = Constants.GrabbyConstants.highZoneExtenderValue;
    liftTarget = Constants.GrabbyConstants.highZoneLifterValue;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("armEncoder", getArmLifterEncoder());
    SmartDashboard.putNumber("leftExtenderEncoder", getRightExtenderEncoder());
    SmartDashboard.putNumber("rightExtenderEncoder", getLeftExtenderEncoder());

    /*if (Math.abs(lowZoneLifterValue - getArmLifterEncoder()) > 0.1) {
      if (getArmLifterEncoder() > lowZoneLifterValue) {
        lowerArm();
      } else if (getArmLifterEncoder() < lowZoneLifterValue) {
          raiseArm();
      } 
    } else {
        stopArmExtender();
    }*/
    if(!manualControl){
     if(liftTarget > getArmLifterEncoder() && liftTarget > liftInit) {
        lifterSpeedCurrent = lifterSpeed;
     }else if (liftTarget < getArmLifterEncoder() && liftTarget < liftInit) {
        lifterSpeedCurrent = -lifterSpeed;
     }else{
         stopArmLifter();
       }

     if(extendTarget < getRightExtenderEncoder() && extendTarget < extendInit) {
      rightExtenderSpeedCurrent = -extenderSpeed; 
       }else if (extendTarget > getRightExtenderEncoder() && extendTarget > extendInit) {
         rightExtenderSpeedCurrent = extenderSpeed;
       }else {
        rightExtenderSpeedCurrent = 0;
        }
    
      if(extendTarget < getLeftExtenderEncoder() && extendTarget < extendInit) {
        leftExtenderSpeedCurrent = -extenderSpeed;
          }else if (extendTarget > getLeftExtenderEncoder() && extendTarget > extendInit) {
            leftExtenderSpeedCurrent = extenderSpeed;
          }else{
              leftExtenderSpeedCurrent = 0;
        }
    }

    if(getRightExtenderEncoder() >= 4){
      if(rightExtenderSpeedCurrent == extenderSpeed){
        rightExtenderSpeedCurrent = 0;
      }
      if(leftExtenderSpeedCurrent == extenderSpeed){
        leftExtenderSpeedCurrent = 0;
      }
    }

    if(getRightLimit()){
      if(rightExtenderSpeedCurrent == -extenderSpeed){
        rightExtenderSpeedCurrent = 0;
      }
      rightArmExtender.setSelectedSensorPosition(0); 
    }

    if(getLeftLimit()){
      if(leftExtenderSpeedCurrent == -extenderSpeed){
        leftExtenderSpeedCurrent = 0;
      }
      leftArmExtender.setSelectedSensorPosition(0); 
    }

    //Make sure right and left extenders are at the same position in manual control
    if(manualControl){
      if(Math.abs(getRightExtenderEncoder()-getLeftExtenderEncoder()) < 0.1){
        //doNothing
      }else if(getRightExtenderEncoder() > getLeftExtenderEncoder()){
        leftExtenderSpeedCurrent = extenderSpeed;
      }else{
        leftExtenderSpeedCurrent = -extenderSpeed;
      }
    }


    armLifter.set(ControlMode.PercentOutput, lifterSpeedCurrent);
    leftArmExtender.set(ControlMode.PercentOutput, leftExtenderSpeedCurrent);
    rightArmExtender.set(ControlMode.PercentOutput, -rightExtenderSpeedCurrent);
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
