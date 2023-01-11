package com.adambots.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.*;
import edu.wpi.first.wpilibj.Solenoid;

import org.opencv.core.*;

import com.adambots.Constants.VisionConstants;

/**
 * Subsystem that only streams camera images to shuffleboard and 
 * does not process the video for any purposes.
 */
public class CameraSubsystem extends SubsystemBase {

    private static UsbCamera frontDetectionCamera;
    private static UsbCamera backDetectionCamera;
    private static CvSink frontCamCvSink;
    private static CvSink backCamCvSink;
    private static Mat frontCamFrame;
    private static Mat backCamFrame;
    private Thread visionThread;
    private Solenoid ringLight;

    public CameraSubsystem(Solenoid ringLight) {
        this.ringLight = ringLight;

        init();
    }

    public void init() {

        ringLight.set(false);
        frontDetectionCamera = CameraServer.startAutomaticCapture(VisionConstants.kFrontCamNumber);
        backDetectionCamera = CameraServer.startAutomaticCapture(VisionConstants.kBackCamNumber);


        frontCamCvSink = CameraServer.getVideo(frontDetectionCamera);
        backCamCvSink = CameraServer.getVideo(backDetectionCamera);
        frontDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, VisionConstants.kFrameWidth, VisionConstants.kFrameHeight, VisionConstants.kProcessingFramesPerSec);
        backDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, VisionConstants.kFrameWidth, VisionConstants.kFrameHeight, VisionConstants.kProcessingFramesPerSec);
        frontCamFrame = new Mat();
        backCamFrame = new Mat();

        visionThread = new Thread(() -> {
            run();
        });
    }

    public void run() {
        // Main vision loop - not really required, but will be useful to print errors etc.
        while (!Thread.interrupted()) {
            if (frontCamCvSink.grabFrame(frontCamFrame) == 0 || 
                backCamCvSink.grabFrame(backCamFrame) == 0) {
                System.out.println("Error in camera server! No frames grabbed");
                continue;
            }
        }
    }

    public Thread getVisionThread() {
        return visionThread;
    }
}