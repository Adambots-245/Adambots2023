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
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.*;
import com.adambots.Constants.VisionConstants;
import com.adambots.Vision.ReflectivePipeline;
import com.adambots.utils.AprilTagReader;
import com.adambots.utils.Log;

public class CameraSubsystem extends SubsystemBase {

  private Solenoid ringLight;
  private ReflectivePipeline reflectiveGrip;
  private static UsbCamera detectionCamera;
  private static Thread visionThread;
  private static CvSink camCvSink;
  private static CvSource processedAprilOutputStream;
  private static CvSource processedReflectedOutputStream;
  private static Mat mat;
  private static Point[] pts = new Point[4];


  public CameraSubsystem(Solenoid ringLight, ReflectivePipeline reflectivePipeline) {
    this.ringLight = ringLight;
    this.reflectiveGrip = reflectivePipeline;
    init();
  }

  public void init() {
    detectionCamera = CameraServer.startAutomaticCapture(0);
    detectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, VisionConstants.kFrameWidth, VisionConstants.kFrameHeight, VisionConstants.kProcessingFramesPerSec);   
    camCvSink = CameraServer.getVideo(detectionCamera);
    processedAprilOutputStream = CameraServer.putVideo("Detected", 640, 480);    processedAprilOutputStream = CameraServer.putVideo("Detected", 640, 480);
    processedReflectedOutputStream = CameraServer.putVideo("Reflected", 640, 480);
    mat = new Mat();
    visionThread = new Thread(() -> {
      run();
    });
  }

  public void run() {

    while (!Thread.interrupted()) {
      if (camCvSink.grabFrame(mat) == 0) {
        processedAprilOutputStream.notifyError(camCvSink.getError());
        System.out.println("Can't Find the Stream reflective");
          continue;
      } 
      try (AprilTagReader tagReader = new AprilTagReader()){
        tagReader.detect(mat);
        System.out.println("Number of tags detected: " + tagReader.count());

        var id = tagReader.getId(0);
        System.out.println("Tags id: " + id);
       // var id2 = tagReader.getId(1);

        var pose = tagReader.getPose(0);
        var rot = pose.getRotation();
        Log.infoF("x=%d, y=%d, z=%d, rX=%d, rY=%d, rZ=%d\n", pose.getX(), pose.getY(), pose.getZ(), rot.getX(), rot.getY(), rot.getZ());
        
        var outlineColor = new Scalar(0, 0, 255);
        Point[] corners = tagReader.getBoundingBox(0);
        for (var i = 0; i <= 3; i++) {
            var j = (i + 1) % 4; 
            Imgproc.line(mat, corners[i], corners[j], outlineColor, 2);
        }
      } catch (Exception e){
        // ignore
      }
      processedAprilOutputStream.putFrame(mat);
      reflectiveGrip.process(mat);
      RotatedRect[] rects = findBoundingBoxes();
      //Draws Finds the largest rect and draws the rectangle
      if (rects.length != 0) {
          RotatedRect rect = findLargestRect(rects);
          draw(rect);
      }
      processedReflectedOutputStream.putFrame(mat);
    }
  }
  public RotatedRect[] findBoundingBoxes() {
    ArrayList<MatOfPoint> contours = reflectiveGrip.filterContoursOutput();
    RotatedRect[] rects = new RotatedRect[contours.size()];
    for (int i = 0; i < contours.size(); i++)
        rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

    return rects;
    }
  public RotatedRect findLargestRect(RotatedRect[] rects) {
    RotatedRect rect = rects[0];
    for (int i = 0; i < rects.length; i++) {
        if (rects[i].size.area() > rect.size.area())
            rect = rects[i];

    }

    return rect;
}

public void draw(RotatedRect rect) {
    rect.points(pts);
    drawRect(pts);
    // findCrosshair(pts);

    // if (crosshair != null)
    //     drawCrosshair();
}

// Draw bounding box around the reflective tape
public void drawRect(Point[] pts) {
    for (int i = 0; i < 4; i++)
        Imgproc.line(mat, pts[i], pts[(i + 1) % 4], VisionConstants.RED, 2);

}

  public Thread getVisionThread() {
    return visionThread;
  }
}