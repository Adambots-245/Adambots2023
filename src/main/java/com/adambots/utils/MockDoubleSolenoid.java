// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/** Add your docs here. */
public class MockDoubleSolenoid {

    private Value status;

    public void set(Value val){
        this.status = val;
    }

    public Value get(){
        return status;
    }
}
