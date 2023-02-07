// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants;

public class ArmTester extends SubsystemBase {

  private final DoubleSolenoid grabby;
  // Solenoid rightGrabby;
  private final TalonFX armTurner;
  private final TalonFX leftArmExtender;
  private final TalonFX rightArmExtender;

  private double lifterSpeed = 0;
  private double extenderSpeed = 0;

  // Set these to actual values from encoder
  private final int lowLevelExtenderPosition = 0;
  private final int midLevelExtenderPosition = 34;
  private final int topLevelExtenderPosition = 59;
  
  private final int lowLevelTurnPosition = 0;
  private final int midLevelTurnPosition = 100;
  private final int topLevelTurnPosition = 200;

  private int extensionLimiter = midLevelExtenderPosition;
  private int turnLimiter = topLevelTurnPosition;

  public enum Position{
    LOW,
    MID,
    TOP
  }

  private Position currentPosition = Position.LOW;
  private final int encoderCPR = Constants.GrabbyConstants.armEncoderCPR;

  public ArmTester(DoubleSolenoid grabby, TalonFX armTurner, TalonFX leftArmExtender, TalonFX rightArmExtender) {
    this.grabby = grabby;
    this.armTurner = armTurner;
    this.leftArmExtender = leftArmExtender;
    this.rightArmExtender = rightArmExtender;

    this.leftArmExtender.setInverted(true);
    this.leftArmExtender.setNeutralMode(NeutralMode.Coast);
    resetToHome();
  }

  public void resetToHome() {
    armTurner.setSelectedSensorPosition(0);
    leftArmExtender.setSelectedSensorPosition(0);
    rightArmExtender.setSelectedSensorPosition(0);

    currentPosition = Position.LOW;
    extensionLimiter = midLevelExtenderPosition;
    turnLimiter = midLevelTurnPosition;
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
    lifterSpeed = GrabbyConstants.lifterSpeed;
    // armTurner.set(ControlMode.PercentOutput, lifterSpeed);
  }

  public void lowerArm() {
    lifterSpeed = -GrabbyConstants.lifterSpeed;
    // armTurner.set(ControlMode.PercentOutput, -lifterSpeed);
  }

  public void stopArmTurner() {
    lifterSpeed = 0;
    //armTurner.set(ControlMode.PercentOutput, 0);
  }

  public void extendArm() {
    extenderSpeed = GrabbyConstants.extenderSpeed;
    // leftArmExtender.set(ControlMode.PercentOutput, -extenderSpeed);
    // rightArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
  }

  public void retractArm() {
    extenderSpeed = -GrabbyConstants.extenderSpeed;
    // leftArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
    // rightArmExtender.set(ControlMode.PercentOutput, -extenderSpeed);
  }

  public void stopArmExtender() {
    extenderSpeed = 0;
    // leftArmExtender.set(ControlMode.PercentOutput, 0);
    // rightArmExtender.set(ControlMode.PercentOutput, 0);
  }

  public double getArmTurnerPosition() {
    return armTurner.getSelectedSensorPosition() / encoderCPR;
  }

  public double getLeftExtenderPosition() {
    return leftArmExtender.getSelectedSensorPosition() / encoderCPR;
  }

  public double getRightExtenderPosition() {
    return rightArmExtender.getSelectedSensorPosition() / encoderCPR;
  }

  public void gotoPosition(Position targetMode){
    if (currentPosition == Position.LOW){
      if (targetMode == Position.MID){
        turnLimiter = midLevelTurnPosition;
        extensionLimiter = midLevelExtenderPosition;
        raiseArm();
        extendArm();
      }
      if (targetMode == Position.TOP){
        turnLimiter = topLevelTurnPosition;
        extensionLimiter = topLevelExtenderPosition;
        raiseArm();
        extendArm();
      }
    }

    if (currentPosition == Position.MID){
      if (targetMode == Position.LOW){
        turnLimiter = lowLevelTurnPosition;
        extensionLimiter = lowLevelExtenderPosition;
        lowerArm();
        retractArm();
      }
      if (targetMode == Position.TOP){
        turnLimiter = topLevelTurnPosition;
        extensionLimiter = topLevelExtenderPosition;
        raiseArm();
        extendArm();
      }
    }

    if (currentPosition == Position.TOP){
      if (targetMode == Position.MID){
        turnLimiter = midLevelTurnPosition;
        extensionLimiter = midLevelExtenderPosition;
        lowerArm();
        retractArm();
      }
      if (targetMode == Position.LOW){
        turnLimiter = lowLevelTurnPosition;
        extensionLimiter = lowLevelExtenderPosition;
        lowerArm();
        retractArm();
      }
    }

    currentPosition = targetMode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    double armPosition = getArmTurnerPosition();
    double rightExtenderPosition = getRightExtenderPosition();
    double leftExtenderPosition = getLeftExtenderPosition();
    int direction = (int) Math.signum(extenderSpeed); // Direction of the arm - assumes that negative is to go down

    SmartDashboard.putNumber("armTurnerEncoder", armPosition);
    SmartDashboard.putNumber("leftExtenderEncoder", leftExtenderPosition);
    SmartDashboard.putNumber("rightExtenderEncoder", rightExtenderPosition);

    // if (armPosition <= 0 || armPosition >= topLevelTurnPosition){
      // stopArmTurner();
    // }

    if (leftExtenderPosition < lowLevelExtenderPosition){ // || rightExtenderPosition <=lowLevelExtenderPosition){
      stopArmExtender();
    }

    if (direction > 0){ // going up
      if (leftExtenderPosition > extensionLimiter) { // || rightExtenderPosition >= extensionLimiter){
        stopArmExtender();
      }
    } else { // going down
      if (leftExtenderPosition < extensionLimiter) { // || rightExtenderPosition <= extensionLimiter){
        stopArmExtender();
      }
    }

    // one more failsafe
    if (leftExtenderPosition > topLevelExtenderPosition){ // || rightExtenderPosition >= topLevelExtenderPosition){
      stopArmExtender();
    }

    SmartDashboard.putNumber("ExtenderSpeed", extenderSpeed);
    // System.out.println("Speed: **********" + extenderSpeed);

    // armTurner.set(ControlMode.PercentOutput, lifterSpeed);
    leftArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
    // rightArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
  }
}
