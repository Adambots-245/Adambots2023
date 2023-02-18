// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.adambots.utils;

import java.util.function.BooleanSupplier;

/** Add your docs here. */
public class MockPhotoEye {
    private boolean isDetecting = false;
    private BooleanSupplier checkStatus;

    public boolean isDetecting(){
        if (checkStatus != null){
            isDetecting = checkStatus.getAsBoolean();
        } else {
            isDetecting = false;
        }

        return isDetecting;
    }

    public void setStatus(BooleanSupplier checkStatus){
        this.checkStatus = checkStatus;
    }
}
