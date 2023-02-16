// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.sensors.PhotoEye;
import com.adambots.utils.ArmMechanism;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabbySubsystem extends SubsystemBase {

  private ArmMechanism armDiagram;

  private final TalonFX armLifter;
  private final TalonFX firstArmExtender;
  private final TalonFX secondArmExtender;
  private final PhotoEye secondExtenderPhotoEye;
  private final PhotoEye firstExtenderPhotoEye;
  private final WPI_CANCoder armRotationEncoder;
  private final DoubleSolenoid grabby;

  private boolean reachedUpperLimit = false;
  private boolean reachedLowerLimit = false;

  public class Position {
    private String name;
    private double armAngleLimit;
    private double firstStageLimit;
    private double secondStageLimit;
    private boolean openGrabby;

    public Position(String name, double armAngleLimit, double firstStageLimit, double secondStageLimit, boolean openGrabby) {
      this.name = name;
      this.armAngleLimit = armAngleLimit;
      this.firstStageLimit = firstStageLimit;
      this.secondStageLimit = secondStageLimit;
      this.openGrabby = openGrabby;
    }

    @Override
    public String toString() {
      return name;
    }
  }

  public Position homePosition = new Position("Home", GrabbyConstants.initiaLifterValue,
      GrabbyConstants.initialFirstExtenderValue,
      GrabbyConstants.initialSecondExtenderValue, false);
  public Position groundPosition = new Position("Ground", GrabbyConstants.groundLifterValue,
      GrabbyConstants.groundFirstExtenderValue,
      GrabbyConstants.groundSecondExtenderValue, true);
  public Position midCubePosition = new Position("Mid-Cube", GrabbyConstants.midCubeLifterValue,
      GrabbyConstants.midCubeFirstExtenderValue,
      GrabbyConstants.midCubeSecondExtenderValue, false);
  public Position highCubePosition = new Position("High-Cube", GrabbyConstants.highCubeLifterValue,
      GrabbyConstants.highCubeFirstExtenderValue, GrabbyConstants.highCubeSecondExtenderValue, false);
  public Position midConePosition = new Position("Mid-cone", GrabbyConstants.midConeLifterValue,
      GrabbyConstants.midConeFirstExtenderValue,
      GrabbyConstants.midConeSecondExtenderValue, false);
  public Position highConePosition = new Position("High-Cone", GrabbyConstants.highConeLifterValue,
      GrabbyConstants.highConeFirstExtenderValue, GrabbyConstants.highConeSecondExtenderValue, false);

  private Position currentPosition = homePosition;
  private Position targetPosition = null;

  private double armSpeed = 0;
  private double firstStageExtenderSpeed = 0;
  private double secondStageExtenderSpeed = 0;
  
  public GrabbySubsystem(TalonFX armLifter, TalonFX firstArmExtender, TalonFX secondArmExtender, 
                          WPI_CANCoder armRotationEncoder, DoubleSolenoid grabby, 
                         PhotoEye secondExtenderPhotoEye, PhotoEye firstExtenderPhotoEye) {

    this.armLifter = armLifter;
    this.firstArmExtender = firstArmExtender;
    this.secondArmExtender = secondArmExtender;
    this.armRotationEncoder = armRotationEncoder;
    this.grabby = grabby;
    this.firstExtenderPhotoEye = firstExtenderPhotoEye;
    this.secondExtenderPhotoEye = secondExtenderPhotoEye;

    // this.armLifter.setInverted(true);
    this.firstArmExtender.setInverted(true);
    this.secondArmExtender.setInverted(true);

    setNeutralMode(NeutralMode.Brake);

    this.currentPosition = homePosition;
    armDiagram = new ArmMechanism(armRotationEncoder.getAbsolutePosition());

  }

  public void setPosition(Position position) {
    targetPosition = position;

    if (targetPosition == null){
      return;
    }

    SmartDashboard.putString("TargetPosition", targetPosition.toString());
    // System.out.println("TargetPosition: " + targetPosition.toString());

    armSpeed = GrabbyConstants.lifterSpeed
        * Math.signum(armRotationEncoder.getAbsolutePosition() - targetPosition.armAngleLimit);

    if (targetPosition.firstStageLimit != 0.0) {
      if(targetPosition.firstStageLimit - firstArmExtender.getSelectedSensorPosition() > 0){
        extendFirstStage();  
      }else{
        retractFirstStage();
      }
    } else if (!firstExtenderPhotoEye.isDetecting()) {
      retractFirstStage();
    }

    if (targetPosition.secondStageLimit != 0.0) {
      if(targetPosition.secondStageLimit - secondArmExtender.getSelectedSensorPosition() > 0){
        extendSecondStage();
      }else{
        retractSecondStage();
      }
    } else if (!secondExtenderPhotoEye.isDetecting()) {
      retractSecondStage();
    }

    if (targetPosition.openGrabby) {
      openGrabby();
    } else {
      closeGrabby();
    }
  }

  public void armUp() {
    armSpeed = -GrabbyConstants.lifterSpeed;
  }

  public void armDown() {
    armSpeed = GrabbyConstants.lifterSpeed;
  }

  public void stopArm(){
    armSpeed = 0;
  }

  public void openGrabby() {
    armDiagram.openClaw();
    grabby.set(Value.kForward);
  }

  public void closeGrabby() {
    armDiagram.closeClaw();
    grabby.set(Value.kReverse);
  }

  public void extendFirstStage() {
    firstStageExtenderSpeed = GrabbyConstants.extenderSpeed;
  }

  public void retractFirstStage() {
    firstStageExtenderSpeed = -GrabbyConstants.extenderSpeed;
  }

  public void stopFirstStage(){
    firstStageExtenderSpeed = 0;
  }

  public void extendSecondStage() {
    secondStageExtenderSpeed = GrabbyConstants.extenderSpeed;
  }

  public void retractSecondStage() {
    secondStageExtenderSpeed = -GrabbyConstants.extenderSpeed;
  }

  public void stopSecondStage(){
    secondStageExtenderSpeed = 0;
  }

  @Override
  public void periodic() {
    setNeutralMode(NeutralMode.Brake);

    SmartDashboard.putNumber("First Stage Encoder", firstArmExtender.getSelectedSensorPosition());
    SmartDashboard.putNumber("Second Stage Encoder", secondArmExtender.getSelectedSensorPosition());
    SmartDashboard.putNumber("Cancoder", armRotationEncoder.getAbsolutePosition());

    // currentPosition = getCurrentPosition();
    if (targetPosition != null) {
      checkPosition();
    }

    reachedLowerLimit = false;
    reachedUpperLimit = false;
    failsafe();

    // if (armSpeed != 0)
    //   System.out.println("ArmSpeed: " + armSpeed);

    armLifter.set(ControlMode.PercentOutput, armSpeed);
    // System.out.println("AP: " + armRotationEncoder.getAbsolutePosition());
    firstArmExtender.set(ControlMode.PercentOutput, firstStageExtenderSpeed);
    secondArmExtender.set(ControlMode.PercentOutput, secondStageExtenderSpeed);
    setMech();
  }

  // Check the position only if a Target Position is set
  private void checkPosition() {

    if (targetPosition == null){
      return;
    }
    
    boolean completed = false;

    // Arm going up - if it has reached the angle limit (encoder) as per the target stop the motor
    if (armSpeed < 0 && armRotationEncoder.getAbsolutePosition() >= targetPosition.armAngleLimit) {
      armSpeed = 0;

      System.out.println("Stopping Arm");

      // Set completed to true only if the extender has also stopped - it may still be running
      if (firstStageExtenderSpeed == 0 && secondStageExtenderSpeed == 0) {
        completed = true;
      }
    }

    // Arm going down - if it has reached the anle limit (encoder) as per the target stop the motor
    if (armSpeed > 0 && armRotationEncoder.getAbsolutePosition() <= targetPosition.armAngleLimit) {
      armSpeed = 0;

      System.out.println("Stopping Arm");

      // Set completed to true only if the extender has also stopped - it may still be running
      if (firstStageExtenderSpeed == 0 && secondStageExtenderSpeed == 0) {
        completed = true;
      }
    }

    // First stage is being extended - stop once the encoder position has been reached
    // No checks for retraction - will be handled by FailSafe
    if (firstStageExtenderSpeed > 0 && firstArmExtender.getSelectedSensorPosition() > targetPosition.firstStageLimit) {
      firstStageExtenderSpeed = 0;

      System.out.println("Stopping First Stage");

      // if arm is not running then set completed to true - it may still be running
      if (armSpeed == 0) {
        completed = true;
      }
    }

    // First stage is being retracted - stop once the encoder position has been reached
    if (firstStageExtenderSpeed < 0 && firstArmExtender.getSelectedSensorPosition() < targetPosition.firstStageLimit && targetPosition.firstStageLimit != 0) {
      firstStageExtenderSpeed = 0;

      System.out.println("Stopping First Stage");

      // if arm is not running then set completed to true - it may still be running
      if (armSpeed == 0) {
        completed = true;
      }
    }

    // Second stage is being extended - stop once the encoder position has been reached
    // No checks for retraction - will be handled by FailSafe
    if (secondStageExtenderSpeed > 0
        && secondArmExtender.getSelectedSensorPosition() > targetPosition.secondStageLimit) {
      secondStageExtenderSpeed = 0;

      System.out.println("Stopping Second Stage");

      if (armSpeed == 0) {
        completed = true;
      }
    }

    // Second stage is being retracted - stop once the encoder position has been reached
    if (secondStageExtenderSpeed < 0 && secondArmExtender.getSelectedSensorPosition() < targetPosition.secondStageLimit && targetPosition.secondStageLimit != 0) {
      secondStageExtenderSpeed = 0;

      System.out.println("Stopping First Stage");

      // if arm is not running then set completed to true - it may still be running
      if (armSpeed == 0) {
        completed = true;
      }
    }

    // All done - set the targetPosition to null so that we can control it manually
    if (completed) {

      System.out.println("Completed. Target is set to Null");

      currentPosition = targetPosition;
      targetPosition = null;
    }
  }

  // Update the diagrams
  private void setMech() {

    if (armSpeed != 0) {
      armDiagram.setArmAngle(armRotationEncoder.getAbsolutePosition());
    }

    if (firstStageExtenderSpeed != 0) {
      if (firstStageExtenderSpeed > 0) {
        armDiagram.extendFirstStage();
      } else {
        armDiagram.retractFirstStage();
      }
    }

    if (secondStageExtenderSpeed != 0) {
      if (secondStageExtenderSpeed > 0) {
        armDiagram.extendSecondStage();
      } else {
        armDiagram.retractSecondStage();
      }
    }
  }

  // Ensure that none of the motors get extended beyond its limits
  private void failsafe() {

    // Ensure that the arm does not go beyond its lowest limit, when driven
    // upwards
    // Also ensure that negative speeds indicate upwards movement
    if (armSpeed < 0 && isArmAtUpperLimit()) {
      armSpeed = 0;
    }

    if (armSpeed >= 0 && isArmAtLowerLimit()) {
      armSpeed = 0;
    }

    if (firstStageExtenderSpeed < 0 && firstExtenderPhotoEye.isDetecting()) {
      firstStageExtenderSpeed = 0;
      firstArmExtender.setSelectedSensorPosition(0);
    }

    if (firstStageExtenderSpeed > 0 && isAtFirstExtenderLimit()) {
      firstStageExtenderSpeed = 0;
    }

    if (secondStageExtenderSpeed < 0 && secondExtenderPhotoEye.isDetecting()) {
      secondStageExtenderSpeed = 0;
      secondArmExtender.setSelectedSensorPosition(0);
    }

    if (secondStageExtenderSpeed > 0 && isAtSecondExtenderLimit()) {
      secondStageExtenderSpeed = 0;
    }

    // System.out.println("FSE Speed: " + firstStageExtenderSpeed);
  }

  private boolean isAtFirstExtenderLimit() {
    return (firstArmExtender.getSelectedSensorPosition() > GrabbyConstants.firstExtenderMaxExtend);
  }

  private boolean isAtSecondExtenderLimit() {
    return (secondArmExtender.getSelectedSensorPosition() > GrabbyConstants.secondExtenderMaxExtend);
  }

  private boolean isArmAtUpperLimit() {
    return (armRotationEncoder.getAbsolutePosition() >= GrabbyConstants.initiaLifterValue);
  }

  private boolean isArmAtLowerLimit() {
    return (armRotationEncoder.getAbsolutePosition() <= GrabbyConstants.groundLifterValue);
  }

  public void setNeutralMode(NeutralMode mode) {
    this.armLifter.setNeutralMode(mode);
    this.firstArmExtender.setNeutralMode(mode);
    this.secondArmExtender.setNeutralMode(mode);
  }
}