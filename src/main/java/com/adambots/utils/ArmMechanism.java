// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import com.adambots.Constants.GrabbyConstants;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class ArmMechanism {

    private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
    private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 20);
    private final MechanismLigament2d m_armTower = m_armPivot
            .append(new MechanismLigament2d("ArmTower", 20, -90, 6, new Color8Bit(Color.kBlue)));
    private final MechanismLigament2d m_arm;
    private final MechanismLigament2d m_firstStageExtender;
    private final MechanismLigament2d m_secondStageExtender;
    private final MechanismLigament2d m_clawUpper;
    private final MechanismLigament2d m_clawLower;

    public ArmMechanism(double initialArmAngle) {

        m_arm = m_armPivot.append(
        new MechanismLigament2d(
            "Arm",
            20,
            initialArmAngle + GrabbyConstants.mech2dAdjustment,
            6,
            new Color8Bit(Color.kYellow)));

    m_firstStageExtender = m_armPivot.append(
        new MechanismLigament2d(
            "FirstStage",
            -10,
            initialArmAngle + GrabbyConstants.mech2dAdjustment,
            6,
            new Color8Bit(Color.kOrange)));

    m_secondStageExtender = m_firstStageExtender.append(
      new MechanismLigament2d(
        "SecondStage",
        -10,
        0,
        6,
        new Color8Bit(Color.kRed)
      )
    );

    m_clawUpper = m_secondStageExtender.append(
      new MechanismLigament2d("Upper Jaw", -3, -5, 3, new Color8Bit(Color.kWhite))
    );

    m_clawLower = m_secondStageExtender.append(
      new MechanismLigament2d("Lower Jaw", -3, 10, 3, new Color8Bit(Color.kWhite))
    );

    SmartDashboard.putData("Arm Sim", m_mech2d);
    }

    public void openClaw(){
        m_clawUpper.setAngle(-15);
        m_clawLower.setAngle(30);
    }

    public void closeClaw(){
        m_clawUpper.setAngle(-5);
        m_clawLower.setAngle(10);
    }

    public void setArmAngle(double angle){
        m_arm.setAngle(angle + GrabbyConstants.mech2dAdjustment);
        m_firstStageExtender.setAngle(angle + GrabbyConstants.mech2dAdjustment);
    }

    public void extendFirstStage(){
        double armLength1 = m_firstStageExtender.getLength();
        m_firstStageExtender.setLength(armLength1 - 0.1);
    }
    
    public void retractFirstStage(){
        double armLength1 = m_firstStageExtender.getLength();
        m_firstStageExtender.setLength(armLength1 + 0.1);
    }

    public void extendSecondStage(){
        double armLength2 = m_secondStageExtender.getLength();
        m_secondStageExtender.setLength(armLength2 - 0.1);
    }
    
    public void retractSecondStage(){
        double armLength2 = m_secondStageExtender.getLength();
        m_secondStageExtender.setLength(armLength2 + 0.1);
    }
}
