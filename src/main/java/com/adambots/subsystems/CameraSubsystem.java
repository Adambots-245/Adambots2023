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

    private static UsbCamera ballDetectionCamera;
    private static CvSink ballCvSink;
    // private static Mat hubMat;
    private Thread visionThread;

    public CameraSubsystem() {
        init();
    }
     
    public void init() {

        try {
            ballDetectionCamera = CameraServer.startAutomaticCapture(0);
            ballCvSink = CameraServer.getVideo(ballDetectionCamera);
            ballDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, VisionConstants.kFrameHeight, VisionConstants.kFrameHeight, VisionConstants.kProcessingFramesPerSec);

            visionThread = new Thread(() -> {
                run();
            });
        } catch (Exception e) {
            System.out.println("Video Camera Error: " + e.getMessage());
        }
    }

    public void run() {
        // Main vision loop - not really required, but will be useful to print errors etc.
        while (!Thread.interrupted()) {
        //     if (hubCvSink.grabFrame(hubMat) == 0) {
        //         System.out.println("Error in camera server! No frames grabbed");
        //         // processedOutputStreamHub.notifyError(hubCvSink.getError());
        //         continue;
        //     }
        }
    }

    public Thread getVisionThread() {
        return visionThread;
    }
}