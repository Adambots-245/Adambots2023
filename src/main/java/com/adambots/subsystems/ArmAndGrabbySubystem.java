// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants;
import com.adambots.Constants.GrabbyConstants.State;
import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.PhotoEye;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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
private final TalonFX firstArmExtender;
private final TalonFX secondArmExtender;
private final PhotoEye secondExtenderPhotoEye;
private final PhotoEye firstExtenderPhotoEye;
private final CANCoder armRotationEncoder;

private double lifterSpeed = Constants.GrabbyConstants.lifterSpeed;
private double extenderSpeed = Constants.GrabbyConstants.extenderSpeed;

private boolean manualControl = false;
private double lifterSpeedCurrent = 0;
private double secondExtenderSpeedCurrent = 0;
private double firstExtenderSpeedCurrent = 0;

public double armTarget = Constants.GrabbyConstants.initiaLifterValue;
public double firstExtenderTarget = Constants.GrabbyConstants.initialFirstExtenderValue;
public double secondExtenderTarget = Constants.GrabbyConstants.initialSecondExtenderValue;

private boolean manual = true;

private boolean armErrorFlag = false;
private boolean firstExtenderErrorFlag = false;
private boolean secondExtenderErrorFlag = false;

public State initState = Constants.GrabbyConstants.initState;
public State groundState = Constants.GrabbyConstants.groundState;
public State midCubeState = Constants.GrabbyConstants.midCubeState;
public State midConeState = Constants.GrabbyConstants.midConeState;
public State highCubeState = Constants.GrabbyConstants.highCubeState;
public State highConeState = Constants.GrabbyConstants.highConeState;

private State currentState = new State(initState.getArmLiftTarget(), initState.getFirstExtendTarget(), initState.getSecondExtendTarget());

private final int armEncoderCPR = Constants.GrabbyConstants.armEncoderCPR;

  public ArmAndGrabbySubystem(DoubleSolenoid grabby, /*Solenoid rightGrabby,*/ TalonFX armLifter, TalonFX firstArmExtender, TalonFX secondArmExtender, PhotoEye secondExtenderPhotoEye, PhotoEye firstExtenderPhotoEye, CANCoder armRotationEncoder) {
    this.grabby = grabby;
   // this.rightGrabby = rightGrabby;
    this.armLifter = armLifter;
    this.firstArmExtender = firstArmExtender;
    this.secondArmExtender = secondArmExtender;
    this.firstExtenderPhotoEye = firstExtenderPhotoEye;
    this.secondExtenderPhotoEye = secondExtenderPhotoEye;
    this.armRotationEncoder = armRotationEncoder;
    
    armLifter.setSelectedSensorPosition(initState.getArmLiftTarget());
    firstArmExtender.setSelectedSensorPosition(initState.getFirstExtendTarget());
    secondArmExtender.setSelectedSensorPosition(initState.getSecondExtendTarget());  
  }

  
  // public void setUpperTargetState(State targetState) {
  //   this.targetState = targetState;
  // }

  // public void setLowerTargetState(State targetState) {
  //   this.targetState = targetState;
  // }

  public void setTargetState(State targetState) {
    armErrorFlag = true;
    firstExtenderErrorFlag = true;
    secondExtenderErrorFlag = true;

    armTarget = targetState.getArmLiftTarget();
    firstExtenderTarget = targetState.getFirstExtendTarget();
    secondExtenderTarget = targetState.getSecondExtendTarget();
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

  // public void extendArm() {
  //   manualControl = true;
  //   secondExtenderSpeedCurrent = extenderSpeed;
  //   firstExtenderSpeedCurrent = extenderSpeed;

  //   //leftArmExtender.set(ControlMode.PercentOutput, -extenderSpeed);
  //   //rightArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
  // }

  // public void retractArm() {
  //   manualControl = true;
  //   secondExtenderSpeedCurrent = -extenderSpeed;
  //   firstExtenderSpeedCurrent = -extenderSpeed;
  //   //leftArmExtender.set(ControlMode.PercentOutput, extenderSpeed);
  //   //rightArmExtender.set(ControlMode.PercentOutput, -extenderSpeed);
  // }

  public void stopArmExtender() {
    secondExtenderSpeedCurrent = 0;
    firstExtenderSpeedCurrent = 0;
    //leftArmExtender.set(ControlMode.PercentOutput, 0);
    //rightArmExtender.set(ControlMode.PercentOutput, 0);
  }

  public double getRightExtenderEncoder(){
    return secondArmExtender.getSelectedSensorPosition() / Constants.GrabbyConstants.armEncoderCPR;
  }

  public double getLeftExtenderEncoder(){
    return firstArmExtender.getSelectedSensorPosition() / Constants.GrabbyConstants.armEncoderCPR;
  }

  public double getArmLifterEncoder(){
    return (armLifter.getSelectedSensorPosition() / Constants.GrabbyConstants.armEncoderCPR);
  }

  public boolean getRightLimit(){
    return secondExtenderPhotoEye.isDetecting();
  }
  
  public boolean getLeftLimit(){
    return firstExtenderPhotoEye.isDetecting();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /*if (Math.abs(lowZoneLifterValue - getArmLifterEncoder()) > 0.1) {
      if (getArmLifterEncoder() > lowZoneLifterValue) {
        lowerArm();
      } else if (getArmLifterEncoder() < lowZoneLifterValue) {
          raiseArm();
      } 
    } else {
        stopArmExtender();
    }*/

    // setCurrentState();
    // setSpeedValues();

    // failsafe();
  

    //Make sure right and left extenders are at the same position in manual control

    if(firstExtenderErrorFlag){

      int firstArmDir = 1;
      if(currentState.getFirstExtendTarget() > firstExtenderTarget){
        firstArmDir = -1;
      }

      if(firstArmExtender.getSelectedSensorPosition() * firstArmDir >= firstExtenderTarget * firstArmDir){
        firstExtenderSpeedCurrent = 0;
        firstExtenderErrorFlag = false;
        currentState = new State(currentState.getArmLiftTarget(), firstExtenderTarget, currentState.getSecondExtendTarget());
      } else if(firstArmDir > 0){
        firstExtenderSpeedCurrent = extenderSpeed;
      }else{
        firstExtenderSpeedCurrent = -extenderSpeed;
      }
    }

    if(secondExtenderErrorFlag){

      int secondArmDir = 1;
      if(currentState.getSecondExtendTarget() > secondExtenderTarget){
        secondArmDir = -1;
      }

      if(secondArmExtender.getSelectedSensorPosition() * secondArmDir >= secondExtenderTarget * secondArmDir){
        secondExtenderSpeedCurrent = 0;
        secondExtenderErrorFlag = false;
        currentState = new State(currentState.getArmLiftTarget(), currentState.getFirstExtendTarget(), secondExtenderTarget);
      }else if(secondArmDir > 0){
        secondExtenderSpeedCurrent = extenderSpeed;
      }else{
        secondExtenderSpeedCurrent = -extenderSpeed;
      }
    }

    if(armErrorFlag){

      int liftArmDir = 1;
      if(currentState.getArmLiftTarget() > armTarget){
        liftArmDir = -1;
      }

      if(armLifter.getSelectedSensorPosition() * liftArmDir >= armTarget * liftArmDir){
        lifterSpeedCurrent = 0;
        armErrorFlag = false;
        currentState = new State(armTarget, currentState.getFirstExtendTarget(), currentState.getSecondExtendTarget());
      }else if(liftArmDir > 0){
        lifterSpeedCurrent = lifterSpeed;
      }else{
        lifterSpeedCurrent = -lifterSpeed;
      }
    }

    if(firstExtenderPhotoEye.isDetecting()){
      if(firstExtenderSpeedCurrent < 0){
        firstExtenderSpeedCurrent = 0;
      }
      firstArmExtender.setSelectedSensorPosition(0);
    }

    if(secondExtenderPhotoEye.isDetecting()){
      if(secondExtenderSpeedCurrent < 0){
        secondExtenderSpeedCurrent = 0;
      }
      secondArmExtender.setSelectedSensorPosition(0);
    }

    armLifter.set(ControlMode.PercentOutput, lifterSpeedCurrent);
    firstArmExtender.set(ControlMode.PercentOutput, firstExtenderSpeedCurrent);
    secondArmExtender.set(ControlMode.PercentOutput, secondExtenderSpeedCurrent);
    System.out.println("2nd Encoder: " + secondArmExtender.getSelectedSensorPosition());
     /*
      if (getRightExtenderEncoder() > Constants.GrabbyConstants.lowZoneExtenderValue) {
        if (Math.abs(Constants.GrabbyConstants.lowZoneLifterValue - getArmLifterEncoder()) > 1) {
           lowerArm(0.1);
         } else if (getArmLifterEncoder() > Constants.GrabbyConstants.lowZoneLifterValue) {
           raiseArm(0.1);
         }
      */

  }


  private void setSpeedValues() {
    // if (currentState == Constants.initState){
      // speed = 0.5;
    // }
  }


  private void setCurrentState() {
    // check photoeye values and decide which state it should be in

    // if (homePhotoEye.get()) {
    //   currentState = Constants.initState;
    //}
  }

}