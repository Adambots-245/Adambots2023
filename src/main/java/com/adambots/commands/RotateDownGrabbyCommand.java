// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.commands;

import com.adambots.actuators.StepperMotor;
import com.adambots.subsystems.GrabSubsystem;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateDownGrabbyCommand extends CommandBase {
  private GrabSubsystem grabSubsystem;
  private StepperMotor motor;

  /** Creates a new RotateGrabbyCommand. */
  public RotateDownGrabbyCommand(GrabSubsystem grabSubsystem, StepperMotor motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(grabSubsystem);
    this.grabSubsystem = grabSubsystem;
    this.motor = motor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    motor.setIdleMode(IdleMode.kBrake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    grabSubsystem.stepDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motor.setIdleMode(IdleMode.kCoast);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
