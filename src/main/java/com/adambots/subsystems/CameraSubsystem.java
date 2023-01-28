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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import java.util.ArrayList;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.*;
import com.adambots.Constants.VisionConstants;
import com.adambots.Vision.ConePipeline;
import com.adambots.Vision.CubePipeline;
import com.adambots.Vision.ReflectivePipeline;


public class CameraSubsystem extends SubsystemBase {

    // delcares all the variables
    private static UsbCamera reflectivetDetectionCamera;
    private static ReflectivePipeline reflectiveGrip;
    private static UsbCamera downDetectionCamera;
    private static CvSink reflectiveCamCvSink;
    private static CvSink downCamCvSink;
    private static CvSource reflectiveOutputStream;
    private static CvSource downOutputStream;
    private static Mat reflectiveMat;
    private Thread visionThread;
    private Solenoid ringLight;
    private static Point crosshair;
    private Object lock = new Object();
    private static Point[] pts = new Point[4];
    private static int pixelDistance;
    private static double angle;
    private NetworkTableEntry angleEntry;
    private static ConePipeline coneGrip;
    private static CubePipeline cubeGrip;
    // delcares the class everything 
    public CameraSubsystem(Solenoid ringLight, ReflectivePipeline reflectivePipeline) {
        this.ringLight = ringLight;

        this.reflectiveGrip = reflectivePipeline;
        init();
    }

    public void init() {
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
        FindAprilTag();
        visionThread = new Thread(() -> {
          run();
        });
    }


    public void run() {
      // Main vision loop
      int frameCount = 0;
      while (!Thread.interrupted()) {
          crosshair = null;
          // checks for errors
          if (reflectiveCamCvSink.grabFrame(reflectiveMat) == 0) {
            reflectiveOutputStream.notifyError(reflectiveCamCvSink.getError());
            System.out.println("Can't Find the Stream reflective");
              continue;
          } 
          // Dont need to mess with this function that much
          FindAprilTag();
          //Processes Reflective Mat
          reflectiveGrip.process(reflectiveMat);
          //Finds all the recangles in the stream
          RotatedRect[] rects = findBoundingBoxes();
          // Draws Finds the largest rect and draws the rectangle
          if (rects.length != 0) {
              RotatedRect rect = findLargestRect(rects);
              draw(rect);
          }
          // checks if there is a crosshair and then calculates the angle
          if (crosshair != null) {
              synchronized (lock) {
                  calculateAngle();

              }
              
          }
        // Puts the frame into the reclative output stream
        reflectiveOutputStream.putFrame(reflectiveMat);
      }

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
      findCrosshair(pts);

      if (crosshair != null)
          drawCrosshair();

  }

  // Draw bounding box around the reflective tape
  public void drawRect(Point[] pts) {
      for (int i = 0; i < 4; i++)
          Imgproc.line(reflectiveMat, pts[i], pts[(i + 1) % 4], VisionConstants.GREEN, 1);

  }

  // Calculate the crosshair position
  public void findCrosshair(Point[] pts) {
      // i is starting point for line, j is next point
      int j;
      for (int i = 0; i < 4; i++) {
          j = (i + 1) % 4;
          if (crosshair == null || (pts[i].y + pts[j].y) / 2 < crosshair.y)
              crosshair = new Point((pts[i].x + pts[j].x) / 2, (pts[i].y + pts[j].y) / 2);

      }

  }

  // Draw the crosshair on the frame
  public void drawCrosshair() {
      Imgproc.line(reflectiveMat, new Point(crosshair.x - 5, crosshair.y - 5), new Point(crosshair.x + 5, crosshair.y + 5), VisionConstants.RED, 3);
      Imgproc.line(reflectiveMat, new Point(crosshair.x - 5, crosshair.y + 5), new Point(crosshair.x + 5, crosshair.y - 5), VisionConstants.RED, 3);

  }

  // Calculate horizontal turret angle
  public void calculateAngle() {
      pixelDistance = (int) crosshair.x - VisionConstants.IMG_HOR_MID;
      angle = pixelDistance * VisionConstants.HOR_DEGREES_PER_PIXEL;
      angleEntry.setDouble(angle);

  }

    public void FindAprilTag() {
        var detector = new AprilTagDetector();
        // look for tag16h5, don't correct any error bits
        detector.addFamily("tag16h5", 0);
   
        // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
        var poseEstConfig =
            new AprilTagPoseEstimator.Config(
                0.1524, 699.3778103158814, 677.7161226393544, 345.6059345433618, 207.12741326228522);
        var estimator = new AprilTagPoseEstimator(poseEstConfig);
        var mat = new Mat();
        var grayMat = new Mat();
        var reflectiveMat = new Mat();
   
        // Instantiate once
        ArrayList<Long> tags = new ArrayList<>();
        var outlineColor = new Scalar(0, 255, 0);
        var crossColor = new Scalar(0, 0, 255);
   
        // We'll output to NT
        NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
        IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();
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
        

    public RotatedRect[] findBoundingBoxes() {
    ArrayList<MatOfPoint> contours = reflectiveGrip.filterContoursOutput();
    RotatedRect[] rects = new RotatedRect[contours.size()];
    for (int i = 0; i < contours.size(); i++)
        rects[i] = Imgproc.minAreaRect(new MatOfPoint2f(contours.get(i).toArray()));

    return rects;
    }
    public Thread getVisionThread() {
        return visionThread;
    }
    
}