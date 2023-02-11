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
import com.ctre.phoenix.sensors.WPI_CANCoder;

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
private final WPI_CANCoder armRotationEncoder;

private double lifterSpeed = Constants.GrabbyConstants.lifterSpeed;
private double extenderSpeed = Constants.GrabbyConstants.extenderSpeed;

private boolean manualControl = false;
private boolean grabCube = true; //this is true if robot grabs a cube
private int midHeight = 1; //this is true if robot is at mid height
private double lifterSpeedCurrent = 0;
private double secondExtenderSpeedCurrent = 0;
private double firstExtenderSpeedCurrent = 0;

public double armTarget = Constants.GrabbyConstants.initiaLifterValue;
public double firstExtenderTarget = Constants.GrabbyConstants.initialFirstExtenderValue;
public double secondExtenderTarget = Constants.GrabbyConstants.initialSecondExtenderValue;

private boolean manual = true;

private int liftArmDir;
private int firstArmDir;
private int secondArmDir;

public State initState = Constants.GrabbyConstants.initState;
public State groundState = Constants.GrabbyConstants.groundState;
public State midCubeState = Constants.GrabbyConstants.midCubeState;
public State midConeState = Constants.GrabbyConstants.midConeState;
public State highCubeState = Constants.GrabbyConstants.highCubeState;
public State highConeState = Constants.GrabbyConstants.highConeState;

private State currentState = new State(initState.getArmLiftTarget(), initState.getFirstExtendTarget(), initState.getSecondExtendTarget());

private final int armEncoderCPR = Constants.GrabbyConstants.armEncoderCPR;

  public ArmAndGrabbySubystem(DoubleSolenoid grabby, /*Solenoid rightGrabby,*/ TalonFX armLifter, TalonFX firstArmExtender, TalonFX secondArmExtender, PhotoEye secondExtenderPhotoEye, PhotoEye firstExtenderPhotoEye, WPI_CANCoder armRotationEncoder) {
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
    SmartDashboard.putNumber("firstExtender", getLeftExtenderEncoder());
    SmartDashboard.putNumber("secondExtender", getRightExtenderEncoder());
    SmartDashboard.putNumber("rotationEncoder", getArmLifterEncoder());


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

      firstArmDir = 0;
      if(currentState.getFirstExtendTarget() > firstExtenderTarget){
        firstArmDir = -1;
      }else if(currentState.getSecondExtendTarget() < secondExtenderTarget){
        firstArmDir = 1;
      }

      if(getLeftExtenderEncoder() * firstArmDir >= firstExtenderTarget * firstArmDir){
        firstExtenderSpeedCurrent = 0;
        currentState = new State(currentState.getArmLiftTarget(), firstExtenderTarget, currentState.getSecondExtendTarget());
      } else if(firstArmDir > 0){
        firstExtenderSpeedCurrent = extenderSpeed;
      }else{
        firstExtenderSpeedCurrent = -extenderSpeed;
      }

      secondArmDir = 0;
      if(currentState.getSecondExtendTarget() > secondExtenderTarget){
        secondArmDir = -1;
      }else if(currentState.getSecondExtendTarget() < secondExtenderTarget){
        secondArmDir = 1;
      }

      if(getRightExtenderEncoder() * secondArmDir >= secondExtenderTarget * secondArmDir){
        secondExtenderSpeedCurrent = 0;
        currentState = new State(currentState.getArmLiftTarget(), currentState.getFirstExtendTarget(), secondExtenderTarget);
      }else if(secondArmDir > 0){
        secondExtenderSpeedCurrent = extenderSpeed;
      }else{
        secondExtenderSpeedCurrent = -extenderSpeed;
      }

      liftArmDir = 0;
      if(currentState.getArmLiftTarget() > armTarget){
        liftArmDir = -1;
      }else if(currentState.getArmLiftTarget() < armTarget){
        liftArmDir = 1;
      }

      if(getArmLifterEncoder() * liftArmDir >= armTarget * liftArmDir){
        lifterSpeedCurrent = 0;
        currentState = new State(armTarget, currentState.getFirstExtendTarget(), currentState.getSecondExtendTarget());
      }else if(liftArmDir > 0){
        lifterSpeedCurrent = lifterSpeed;
      }else if(liftArmDir < 0){
        lifterSpeedCurrent = -lifterSpeed;
      }

    failSafes();

    

    armLifter.set(ControlMode.PercentOutput, lifterSpeedCurrent);
    firstArmExtender.set(ControlMode.PercentOutput, firstExtenderSpeedCurrent);
    secondArmExtender.set(ControlMode.PercentOutput, secondExtenderSpeedCurrent);
    System.out.println("CANCoder: " + armRotationEncoder.getAbsolutePosition());
     /*   
      if (getRightExtenderEncoder() > Constants.GrabbyConstants.lowZoneExtenderValue) {
        if (Math.abs(Constants.GrabbyConstants.lowZoneLifterValue - getArmLifterEncoder()) > 1) {
           lowerArm(0.1);
         } else if (getArmLifterEncoder() > Constants.GrabbyConstants.lowZoneLifterValue) {
           raiseArm(0.1);
         }
      */

  }


  private void failSafes() {

    //First Arm
    if(firstExtenderPhotoEye.isDetecting()){
      if(firstExtenderSpeedCurrent < 0){
        firstExtenderSpeedCurrent = 0;
      }
      firstArmExtender.setSelectedSensorPosition(0);
    }

    if(firstArmExtender.getSelectedSensorPosition() > Constants.GrabbyConstants.firstExtenderMaxExtend){
      if(secondExtenderSpeedCurrent > 0){
        secondExtenderSpeedCurrent = 0;
      }
    }

    //Second Arm
    if(secondExtenderPhotoEye.isDetecting()){
      if(secondExtenderSpeedCurrent < 0){
        secondExtenderSpeedCurrent = 0;
      }
      secondArmExtender.setSelectedSensorPosition(0);
    }

    if(secondArmExtender.getSelectedSensorPosition() > Constants.GrabbyConstants.secondExtenderMaxExtend){
      if(secondExtenderSpeedCurrent > 0){
        secondExtenderSpeedCurrent = 0;
      }
    }

    //Triiiiiiiiiiiiiiiiiiiiiiiiig
    
    double getMaxExtention = (45 / Math.cos(armRotationEncoder.getAbsolutePosition())) * Constants.GrabbyConstants.rotationPerInch;

     if( getMaxExtention < getRightExtenderEncoder() + getLeftExtenderEncoder()){
      secondExtenderSpeedCurrent = -extenderSpeed;
     }


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

  public boolean getCube(){
    return grabCube;
  }
  public int getMidHeight(){
    return midHeight;
  }
  public void setCube(boolean x){
    grabCube = x;
  }
  public void setMidHeight(int y){
    midHeight = y;
  }

}