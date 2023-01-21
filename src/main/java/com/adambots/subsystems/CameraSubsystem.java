package com.adambots.subsystems;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cscore.*;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.*;


import com.adambots.Constants.VisionConstants;


/**
 * Subsystem that only streams camera images to shuffleboard and
 * does not process the video for any purposes.
 */
public class CameraSubsystem extends SubsystemBase {


    private static UsbCamera reflectivetDetectionCamera;
    private static UsbCamera downDetectionCamera;
    private static CvSink reflectiveCamCvSink;
    private static CvSink downCamCvSink;
    private static CvSource reflectiveOutputStream;
    private static CvSource downOutputStream;
    private static Mat frontCamFrame;
    private static Mat reflectiveCamFrame;
    private Thread visionThread;
    private Solenoid ringLight;


    public CameraSubsystem(Solenoid ringLight) {
        this.ringLight = ringLight;


        init();
    }


    public void init() {
        var detector = new AprilTagDetector();
        // look for tag16h5, don't correct any error bits
        detector.addFamily("tag16h5", 0);
   
        // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
        // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
        var poseEstConfig =
            new AprilTagPoseEstimator.Config(
                0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
        var estimator = new AprilTagPoseEstimator(poseEstConfig);
   
        // Get the UsbCamera from CameraServer
        downDetectionCamera = CameraServer.startAutomaticCapture(0);
        reflectivetDetectionCamera = CameraServer.startAutomaticCapture(1);
        // Set the resolution
        downDetectionCamera.setResolution(640, 480);
        reflectivetDetectionCamera.setResolution(640, 480);
   
        // Get a CvSink. This will capture Mats from the camera
        downCamCvSink = CameraServer.getVideo();
        reflectiveCamCvSink = CameraServer.getVideo();
        // Setup a CvSource. This will send images back to the Dashboard
        downOutputStream = CameraServer.putVideo("Detected", 640, 480);
        reflectiveOutputStream = CameraServer.putVideo("Reflected", 640, 480);
   
        // Mats are very memory expensive. Lets reuse these.
        var mat = new Mat();
        var grayMat = new Mat();
   
        // Instantiate once
        ArrayList<Long> tags = new ArrayList<>();
        var outlineColor = new Scalar(0, 255, 0);
        var crossColor = new Scalar(0, 0, 255);
   
        // We'll output to NT
        NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
        IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();
   
        // This cannot be 'true'. The program will never exit if it is. This
        // lets the robot stop this thread when restarting robot code or
        // deploying.
        while (!Thread.interrupted()) {
          // Tell the CvSink to grab a frame from the camera and put it
          // in the source mat.  If there is an error notify the output.
          if (downCamCvSink.grabFrame(mat) == 0) {
            // Send the output the error.
            downOutputStream.notifyError(downCamCvSink.getError());
            // skip the rest of the current iteration
            continue;
          }
   
          Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);
   
          AprilTagDetection[] detections = detector.detect(grayMat);
   
          // have not seen any tags yet
          tags.clear();
   
          for (AprilTagDetection detection : detections) {
            // remember we saw this tag
            tags.add((long) detection.getId());
   
            // draw lines around the tag
            for (var i = 0; i <= 3; i++) {
              var j = (i + 1) % 4;
              var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
              var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
              Imgproc.line(mat, pt1, pt2, outlineColor, 2);
            }
   
            // mark the center of the tag
            var cx = detection.getCenterX();
            var cy = detection.getCenterY();
            var ll = 10;
            Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
            Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);
   
            // identify the tag
            Imgproc.putText(
                mat,
                Integer.toString(detection.getId()),
                new Point(cx + ll, cy),
                Imgproc.FONT_HERSHEY_SIMPLEX,
                1,
                crossColor,
                3);
   
            // determine pose
            Transform3d pose = estimator.estimate(detection);
   
            // put pose into dashbaord
            Rotation3d rot = pose.getRotation();
            tagsTable
                .getEntry("pose_" + detection.getId())
                .setDoubleArray(
                    new double[] {
                      pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ()
                    });
          }
   
          // put list of tags onto dashboard
          pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());
   
          // Give the output stream a new image to display
          downOutputStream.putFrame(mat);
        }
   
        pubTags.close();
        detector.close();
    }
    //     ringLight.set(true);
    //     frontDetectionCamera = CameraServer.startAutomaticCapture(VisionConstants.kFrontCamNumber);
    //     backDetectionCamera = CameraServer.startAutomaticCapture(VisionConstants.kBackCamNumber);




    //     frontCamCvSink = CameraServer.getVideo(frontDetectionCamera);
    //     backCamCvSink = CameraServer.getVideo(backDetectionCamera);
    //     frontDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, VisionConstants.kFrameWidth, VisionConstants.kFrameHeight, VisionConstants.kProcessingFramesPerSec);
    //     backDetectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, VisionConstants.kFrameWidth, VisionConstants.kFrameHeight, VisionConstants.kProcessingFramesPerSec);
    //     frontCamFrame = new Mat();
    //     backCamFrame = new Mat();


    //     visionThread = new Thread(() -> {
    //         run();
    //     });
    // }


    // public void run() {
    //     // Main vision loop - not really required, but will be useful to print errors etc.
    //     while (!Thread.interrupted()) {
    //         if (frontCamCvSink.grabFrame(frontCamFrame) == 0 ||
    //             backCamCvSink.grabFrame(backCamFrame) == 0) {
    //             System.out.println("Error in camera server! No frames grabbed");
    //             continue;
    //         }
    //     }
    // }


    public Thread getVisionThread() {
        return visionThread;
    }
}

