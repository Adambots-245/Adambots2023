// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.subsystems;

import com.adambots.Constants.GrabbyConstants;
import com.adambots.utils.ArmMechanism;
import com.adambots.utils.MockCancoder;
import com.adambots.utils.MockDoubleSolenoid;
import com.adambots.utils.MockMotor;
import com.adambots.utils.MockPhotoEye;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GrabberSubsystem extends SubsystemBase {

  private MockMotor armMotor;
  private MockMotor firstStageMotor;
  private MockMotor secondStageMotor;
  private MockCancoder armCancoder;
  private MockDoubleSolenoid clawSolenoid;
  private MockPhotoEye firstStagePhotoEye;
  private MockPhotoEye secondStagePhotoEye;
  private ArmMechanism armDiagram;

  public class Position {
    private String name;
    private double armAngleLimit;
    private double firstStageLimit;
    private double secondStageLimit;
    private boolean openClaw;

    public Position(String name, double armAngleLimit, double firstStageLimit, double secondStageLimit,
        boolean openClaw) {
      this.name = name;
      this.armAngleLimit = armAngleLimit;
      this.firstStageLimit = firstStageLimit;
      this.secondStageLimit = secondStageLimit;
      this.openClaw = openClaw;
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

  public GrabberSubsystem(MockMotor armMotor, MockMotor firstStageMotor, MockMotor secondStageMotor,
      MockCancoder armCancoder, MockDoubleSolenoid clawSolenoid, MockPhotoEye firstStagePhotoEye,
      MockPhotoEye secondStagePhotoEye) {

    this.armMotor = armMotor;
    this.firstStageMotor = firstStageMotor;
    this.secondStageMotor = secondStageMotor;
    this.armCancoder = armCancoder;
    this.clawSolenoid = clawSolenoid;
    this.firstStagePhotoEye = firstStagePhotoEye;
    this.secondStagePhotoEye = secondStagePhotoEye;

    this.armMotor.setInverted(true);
    this.firstStageMotor.setInverted(true);
    // this.secondStageMotor.setInverted(true);

    setNeutralMode(NeutralMode.Brake);

    this.currentPosition = homePosition;
    armDiagram = new ArmMechanism(armCancoder.getAbsolutePosition());

    firstStagePhotoEye.setStatus(() -> {
      return (firstStageMotor.getSelectedSensorPosition() <= 0.0);
    });

    secondStagePhotoEye.setStatus(() -> {
      return (secondStageMotor.getSelectedSensorPosition() <= 0.0);
    });
  }

  public void setPosition(Position position) {
    targetPosition = position;
    SmartDashboard.putString("TargetPosition", targetPosition.toString());
    // System.out.println("TargetPosition: " + targetPosition.toString());
    armSpeed = GrabbyConstants.lifterSpeed
        * Math.signum(armCancoder.getAbsolutePosition() - targetPosition.armAngleLimit);
    // System.out.println("ASE: " + armCancoder.getAbsolutePosition() + " - " +
    // targetPosition.armAngleLimit);
    // System.out.println("ARM Speed: " + armSpeed);

    if (targetPosition.firstStageLimit != 0.0) {
      if(targetPosition.firstStageLimit - firstStageMotor.getSelectedSensorPosition() > 0){
        extendFirstStage();
      }else{
        retractFirstStage();
      }
    } else if (!firstStagePhotoEye.isDetecting()) {
      retractFirstStage();
    }

    if (targetPosition.secondStageLimit != 0.0) {
      if(targetPosition.secondStageLimit - secondStageMotor.getSelectedSensorPosition() > 0){
        extendSecondStage();
      }
        retractSecondStage();
    } else if (!secondStagePhotoEye.isDetecting()) {
      retractSecondStage();
    }

    if (targetPosition.openClaw) {
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

  public void openGrabby() {
    armDiagram.openClaw();
    clawSolenoid.set(Value.kForward);
  }

  public void closeGrabby() {
    armDiagram.closeClaw();
    clawSolenoid.set(Value.kReverse);
  }

  public void extendFirstStage() {
    firstStageExtenderSpeed = GrabbyConstants.extenderSpeed;
  }

  public void retractFirstStage() {
    firstStageExtenderSpeed = -GrabbyConstants.extenderSpeed;
  }

  public void extendSecondStage() {
    secondStageExtenderSpeed = GrabbyConstants.extenderSpeed;
  }

  public void retractSecondStage() {
    secondStageExtenderSpeed = -GrabbyConstants.extenderSpeed;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Cancoder", armCancoder.getAbsolutePosition());

    // currentPosition = getCurrentPosition();
    if (targetPosition != null) {
      checkPosition();
    }

    failsafe();

    // if (armSpeed != 0)
    //   System.out.println("ArmSpeed: " + armSpeed);

    armMotor.set(ControlMode.PercentOutput, armSpeed);
    // System.out.println("AP: " + armCancoder.getAbsolutePosition());
    firstStageMotor.set(ControlMode.PercentOutput, firstStageExtenderSpeed);
    secondStageMotor.set(ControlMode.PercentOutput, secondStageExtenderSpeed);
    setMech();
  }

  // Check the position only if a Target Position is set
  private void checkPosition() {
    boolean completed = false;

    // Arm going up - if it has reached the angle limit (encoder) as per the target stop the motor
    if (armSpeed < 0 && armCancoder.getAbsolutePosition() >= targetPosition.armAngleLimit) {
      armSpeed = 0;

      System.out.println("Stopping Arm");

      // Set completed to true only if the extender has also stopped - it may still be running
      if (firstStageExtenderSpeed == 0 && secondStageExtenderSpeed == 0) {
        completed = true;
      }
    }

    // Arm going down - if it has reached the anle limit (encoder) as per the target stop the motor
    if (armSpeed > 0 && armCancoder.getAbsolutePosition() <= targetPosition.armAngleLimit) {
      armSpeed = 0;

      System.out.println("Stopping Arm");

      // Set completed to true only if the extender has also stopped - it may still be running
      if (firstStageExtenderSpeed == 0 && secondStageExtenderSpeed == 0) {
        completed = true;
      }
    }

    // First stage is being extended - stop once the encoder position has been reached
    // No checks for retraction - will be handled by FailSafe
    if (firstStageExtenderSpeed > 0 && firstStageMotor.getSelectedSensorPosition() > targetPosition.firstStageLimit) {
      firstStageExtenderSpeed = 0;

      System.out.println("Stopping First Stage");

      // if arm is not running then set completed to true - it may still be running
      if (armSpeed == 0) {
        completed = true;
      }
    }

    // First stage is being retracted - stop once the encoder position has been reached
    if (firstStageExtenderSpeed < 0 && firstStageMotor.getSelectedSensorPosition() < targetPosition.firstStageLimit && targetPosition.firstStageLimit != 0.0) {
      firstStageExtenderSpeed = 0;

      System.out.println("Stopping First Stage");

      // if arm is not running then set completed to true - it may still be running
      if (armSpeed == 0) {
        completed = true;
      }
    }

    // Second stage is being extended - stop once the encoder position has been reached
    // No checks for retraction - will be handled by FailSafe
    if (secondStageExtenderSpeed > 0 && secondStageMotor.getSelectedSensorPosition() > targetPosition.secondStageLimit) {
      secondStageExtenderSpeed = 0;

      System.out.println("Stopping Second Stage");

      if (armSpeed == 0) {
        completed = true;
      }
    }

    // Second stage is being retracted - stop once the encoder position has been reached
    if (secondStageExtenderSpeed < 0 && secondStageMotor.getSelectedSensorPosition() < targetPosition.secondStageLimit && targetPosition.secondStageLimit != 0.0) {
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
      armDiagram.setArmAngle(armCancoder.getAbsolutePosition());
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

    if (firstStageExtenderSpeed < 0 && firstStagePhotoEye.isDetecting()) {
      firstStageExtenderSpeed = 0;
      firstStageMotor.setSelectedSensorPosition(0);
    }

    if (firstStageExtenderSpeed > 0 && isAtFirstExtenderLimit()) {
      firstStageExtenderSpeed = 0;
    }

    if (secondStageExtenderSpeed < 0 && secondStagePhotoEye.isDetecting()) {
      secondStageExtenderSpeed = 0;
      secondStageMotor.setSelectedSensorPosition(0);
    }

    if (secondStageExtenderSpeed > 0 && isAtSecondExtenderLimit()) {
      secondStageExtenderSpeed = 0;
    }

    // System.out.println("FSE Speed: " + firstStageExtenderSpeed);
  }

  private boolean isAtFirstExtenderLimit() {
    return (firstStageMotor.getSelectedSensorPosition() > GrabbyConstants.firstExtenderMaxExtend);
  }

  private boolean isAtSecondExtenderLimit() {
    return (secondStageMotor.getSelectedSensorPosition() > GrabbyConstants.secondExtenderMaxExtend);
  }

  private boolean isArmAtUpperLimit() {
    return (armCancoder.getAbsolutePosition() >= GrabbyConstants.initiaLifterValue);
  }

  private boolean isArmAtLowerLimit() {
    return (armCancoder.getAbsolutePosition() <= GrabbyConstants.groundLifterValue);
  }

  public void setNeutralMode(NeutralMode mode) {
    this.armMotor.setNeutralMode(mode);
    this.firstStageMotor.setNeutralMode(mode);
    this.secondStageMotor.setNeutralMode(mode);
  }
}