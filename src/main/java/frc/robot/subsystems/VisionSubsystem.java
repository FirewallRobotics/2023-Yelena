// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class VisionSubsystem extends SubsystemBase {

  public NetworkTableInstance inst = NetworkTableInstance.getDefault();

  public NetworkTable aprilTag = inst.getTable("apriltag");
  public NetworkTableEntry ntTagCenterX = aprilTag.getEntry("center_x");
  public NetworkTableEntry ntTagCenterY = aprilTag.getEntry("center_y");
  public NetworkTableEntry ntTagRotation = aprilTag.getEntry("rotation");
  public NetworkTableEntry ntTagArea = aprilTag.getEntry("area");

  public NetworkTableEntry ntPose = aprilTag.getEntry("pose");
  public Pose3d pose;

  public NetworkTable cube = inst.getTable("cube");
  public NetworkTableEntry ntCubeCenterX = cube.getEntry("center_x");
  public NetworkTableEntry ntCubeCenterY = cube.getEntry("center_y");
  public NetworkTableEntry ntCubeRadius = cube.getEntry("radius");

  public NetworkTable cone = inst.getTable("cone");
  public NetworkTableEntry ntConeXMin = cone.getEntry("x_min");
  public NetworkTableEntry ntConeXMax = cone.getEntry("x_max");
  public NetworkTableEntry ntConeYMin = cone.getEntry("y_min");
  public NetworkTableEntry ntConeYMax = cone.getEntry("y_max");

  public NetworkTable tape = inst.getTable("reflective_tape");
  public NetworkTableEntry ntTapeCenterX = aprilTag.getEntry("center_x");
  public NetworkTableEntry ntTapeCenterY = aprilTag.getEntry("center_y");
  public NetworkTableEntry ntTapeArea = aprilTag.getEntry("area");

  public double adjustLeftRight;
  public double adjustBackForward;

  public double adjustConePlacementLeftRight;
  public double adjustConePlacementBackForward;

  public double adjustCubePlacementLeftRight;
  public double adjustCubePlacementBackForward;

  public double cameraCenterX = Constants.VisionConstants.kCameraCenterX;
  public double targetCenterXRange = Constants.VisionConstants.kTargetCenterXRange;
  public double coneTargetWidth = Constants.VisionConstants.kConeTargetWidth;
  public double coneTargetWidthRange = Constants.VisionConstants.kConeTargetWidthRange;
  public double cubeTargetRadius = Constants.VisionConstants.kCubeTargetRadius;
  public double cubeTargetRadiusRange = Constants.VisionConstants.kCubeTargetRadiusRange;
  public double tagConeTargetX = Constants.VisionConstants.kAprilTagConeTargetX;
  public double tagConeTargetXRange = Constants.VisionConstants.kAprilTagConeTargetXRange;
  public double tagConeTargetArea = Constants.VisionConstants.kAprilTagConeTargetArea;
  public double tagConeTargetAreaRange = Constants.VisionConstants.kAprilTagConeTargetAreaRange;
  public double tagCubeTargetX = Constants.VisionConstants.kAprilTagCubeTargetX;
  public double tagCubeTargetXRange = Constants.VisionConstants.kAprilTagCubeTargetXRange;
  public double tagCubeTargetArea = Constants.VisionConstants.kAprilTagCubeTargetArea;
  public double tagCubeTargetAreaRange = Constants.VisionConstants.kAprilTagCubeTargetAreaRange;

  public double decelerationDistance = Constants.VisionConstants.kDecelerationDistance;

  public double highestXDifferenceLED = Constants.VisionConstants.kHighestXDifferenceLED;
  public double highestZDifferenceLED = Constants.VisionConstants.kHighestZDifferenceLED;
  public double highestTagAreaDifferenceLED =
      Constants.VisionConstants.kHighestTagAreaDifferenceLED;
  public int LEDProx;

  public VisionSubsystem() {}

  @Override
  public void periodic() {

    double tagCenterX = ntTagCenterX.getDouble(0);
    // double tagCenterY = ntTagCenterY.getDouble(0);
    double tagArea = ntTagArea.getDouble(0);

    // Code for some reason doesn't work unless I make the default list a variable
    double[] defaultArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double[] poseInfo = ntPose.getDoubleArray(defaultArray);

    double cubeCenterX = ntCubeCenterX.getDouble(0);
    // double cubeCenterY = ntCubeCenterY.getDouble(0);
    double cubeRadius = ntCubeRadius.getDouble(0);

    double coneXMin = ntConeXMin.getDouble(0);
    double coneXMax = ntConeXMax.getDouble(0);
    // double coneYMin = ntConeYMin.getDouble(0);
    // double coneYMax = ntConeYMax.getDouble(0);

    double coneWidth = coneXMax - coneXMin;
    double coneCenterX = (coneXMin + coneXMax) / 2.0;

    // Create pose from data collected from NetworkTable

    pose =
        new Pose3d(
            poseInfo[0],
            poseInfo[1],
            poseInfo[2],
            new Rotation3d(poseInfo[3], poseInfo[4], poseInfo[5]));

    // Calculating values

    // Negative if cone is left, positive if cone is right
    double tagConeCenterXDifference = tagCenterX - tagConeTargetX;
    // Negative if too far away, positive if too close
    double tagConeAreaDifference = tagArea - tagConeTargetArea;
    // Negative if cone is left, positive if cone is right
    double tagCubeCenterXDifference = tagCenterX - tagCubeTargetX;
    // Negative if too far away, positive if too close
    double tagCubeAreaDifference = tagArea - tagCubeTargetArea;
    // Negative if too far away, positive if too close
    double coneWidthDifference = coneWidth - coneTargetWidth;
    // Negative if cone is left, positive if cone is right
    double coneCenterDifference = coneCenterX - cameraCenterX;
    // Negative if too far away, positive if too close
    double cubeRadiusDifference = cubeRadius - cubeTargetRadius;
    // Negative if cube is left, positive if cube is right
    double cubeCenterDifference = cubeCenterX - cameraCenterX;

    // Set to lowest
    LEDProx = 0;

    // If statements decide which adjust code runs?

    if (coneWidth > (cubeRadius * 2)) {
      // Cone left or right of robot
      adjustLeftRight = VisionAdjust(coneCenterDifference, targetCenterXRange);
      ChangeLEDProx(coneCenterDifference, targetCenterXRange, highestXDifferenceLED);
      // Distance from cone based on width
      adjustBackForward = VisionAdjust(coneWidthDifference, coneTargetWidthRange);
      ChangeLEDProx(coneWidthDifference, coneTargetWidthRange, highestZDifferenceLED);
    } else {
      // Cube left or right of robot
      adjustLeftRight = VisionAdjust(cubeCenterDifference, targetCenterXRange);
      ChangeLEDProx(cubeCenterDifference, targetCenterXRange, highestXDifferenceLED);
      // Distance from cube based on radius
      adjustBackForward = VisionAdjust(cubeRadiusDifference, cubeTargetRadiusRange);
      ChangeLEDProx(cubeRadiusDifference, cubeTargetRadiusRange, highestZDifferenceLED);
    }

    // Cube tag near target X
    adjustCubePlacementLeftRight = VisionAdjust(tagCubeCenterXDifference, tagCubeTargetXRange);
    ChangeLEDProx(tagCubeCenterXDifference, tagCubeTargetXRange, highestXDifferenceLED);
    // Distance from cube tag based on area
    adjustCubePlacementBackForward = VisionAdjust(tagCubeAreaDifference, tagCubeTargetAreaRange);
    ChangeLEDProx(tagCubeAreaDifference, tagCubeTargetAreaRange, highestTagAreaDifferenceLED);

    // Cone tag near target X
    adjustConePlacementLeftRight = VisionAdjust(tagConeCenterXDifference, tagConeTargetXRange);
    ChangeLEDProx(tagConeCenterXDifference, tagConeTargetXRange, highestXDifferenceLED);
    // Distance from cone tag based on area
    adjustConePlacementBackForward = VisionAdjust(tagConeAreaDifference, tagConeTargetAreaRange);
    ChangeLEDProx(tagConeAreaDifference, tagConeTargetAreaRange, highestTagAreaDifferenceLED);
  }

  private double VisionAdjust(double difference, double targetRange) {
    // Distance from cone based on width
    if (Math.abs(difference) <= targetRange) // Close enough
    {
      return 0.0;
    } else if (difference < 0) // Too far
    {
      return 1 * DecelerationSpeed(difference, targetRange);
    } else // Too close
    {
      return -1 * DecelerationSpeed(difference, targetRange);
    }
  }

  private double DecelerationSpeed(double positionDifference, double targetRange) {
    double distanceFromTarget = Math.abs(positionDifference) - targetRange;
    double speed = (distanceFromTarget / decelerationDistance) * 9.0 / 10.0 + 0.1;

    if (speed < 1) // Max value for speed is 1
    {
      return speed;
    } else {
      return 1.0;
    }
  }

  private void ChangeLEDProx(
      double positionDifference, double targetRange, double highestDifferenceLED) {
    double distanceFromTarget = Math.abs(positionDifference) - targetRange;

    if (distanceFromTarget <= highestDifferenceLED) {
      if (distanceFromTarget > 0) {
        int LEDProxTemp = (int) ((1 - (distanceFromTarget / highestDifferenceLED)) * 25.0);
        if (LEDProxTemp > LEDProx) LEDProx = LEDProxTemp;
      } else {
        LEDProx = 25;
      }
    }
  }

  public int GetLEDProx() {
    return LEDProx;
  }

  public void apriltagVisionThreadProc() {
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
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

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
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
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
      outputStream.putFrame(mat);
    }

    pubTags.close();
    detector.close();
  }
}
