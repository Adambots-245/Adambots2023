package com.adambots.subsystems;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.adambots.Constants.VisionConstants;
import com.adambots.Vision.ConePipeline;
import com.adambots.Vision.CubePipeline;
import com.adambots.Vision.ReflectivePipeline;
import com.adambots.utils.AprilTagReader;
import com.adambots.utils.LimelightHelpers;
import com.adambots.utils.Log;
import com.adambots.utils.LimelightHelpers.LimelightTarget_Retro;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionProcessingSubsystem extends SubsystemBase {

  private ConePipeline coneGrip;
  private CubePipeline cubeGrip;
  private static UsbCamera detectionCamera;
  private static Thread visionThread;
  private static CvSink camCvSink;
  private static CvSource processedCubeOutputStream;
  private static CvSource processedConeOutputStream;
  private static Mat mat;
  private static Point[] pts = new Point[4];
  private final static Field2d aprilTagField = new Field2d();
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("botpose");
  private MedianFilter Filter;
  private MedianFilter hubMinYFilter;
  // private LimelightHelpers limelightHelper = new LimelightHelpers();


  public VisionProcessingSubsystem(ConePipeline conePipeline, CubePipeline cubePipeline) {
    this.coneGrip = conePipeline;
    this.cubeGrip = cubePipeline;
    init();
  }

  public void init() {
     
    detectionCamera = CameraServer.startAutomaticCapture(0);
    detectionCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, VisionConstants.kFrameWidth, VisionConstants.kFrameHeight, VisionConstants.kProcessingFramesPerSec);   
    camCvSink = CameraServer.getVideo(detectionCamera);
    processedCubeOutputStream = CameraServer.putVideo("Cube", 320, 240);
    processedConeOutputStream = CameraServer.putVideo("Cone", 320, 240);

    mat = new Mat();
    SmartDashboard.putData("April Tag Field", aprilTagField);
    visionThread = new Thread(() -> {
      run2();
    });
  }

  public void run2() {

    while (!Thread.interrupted()) {
      if (camCvSink.grabFrame(mat) == 0) {
        processedCubeOutputStream.notifyError(camCvSink.getError());
        System.out.println("Can't Find the Stream reflective");
          continue;
      } 
      cubeGrip.process(mat);
      processedCubeOutputStream.putFrame(mat);
      coneGrip.process(mat);
      processedConeOutputStream.putFrame(mat);
      aprilTagField.setRobotPose(getPose());
    }
  }
  public Pose2d getPose() {
    // double [] def = {0,0,0,0,0};
    // double[] poseValues = tx.getDoubleArray(def);
    //System.out.println(poseValues[0]);
  // Translation using z and x
  // Translation2d t2d = new Translation2d(-poseValues[1], poseValues[0]);
  // Rotation using ry
  // Rotation2d r2d = new Rotation2d(poseValues[4]);
  Pose2d pose = LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue();
  return pose; 
}
  public RotatedRect[] findBoundingBoxes() {
    ArrayList<MatOfPoint> contours = coneGrip.filterContoursOutput();
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
}

// Draw bounding box around the reflective tape
public void drawRect(Point[] pts) {
    for (int i = 0; i < 4; i++)
        Imgproc.line(mat, pts[i], pts[(i + 1) % 4], VisionConstants.RED, 2);

}

public static Field2d getAprilField2d() {
  return aprilTagField;
}

  public Thread getVisionThread() {
    return visionThread;
  }
}