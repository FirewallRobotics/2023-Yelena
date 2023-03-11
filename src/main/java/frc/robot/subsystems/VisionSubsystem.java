// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  public double highestTagAreaDifferenceLED = Constants.VisionConstants.kHighestTagAreaDifferenceLED;
  public int LEDProx;

  public VisionSubsystem() {}

  @Override
  public void periodic() {

    double tagCenterX = ntTagCenterX.getDouble(0);
    double tagCenterY = ntTagCenterY.getDouble(0);
    double tagArea = ntTagArea.getDouble(0);

    // Code for some reason doesn't work unless I make the default list a variable
    double[] defaultArray = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double[] poseInfo = ntPose.getDoubleArray(defaultArray);

    double cubeCenterX = ntCubeCenterX.getDouble(0);
    double cubeCenterY = ntCubeCenterY.getDouble(0);
    double cubeRadius = ntCubeRadius.getDouble(0);

    double coneXMin = ntConeXMin.getDouble(0);
    double coneXMax = ntConeXMax.getDouble(0);
    double coneYMin = ntConeYMin.getDouble(0);
    double coneYMax = ntConeYMax.getDouble(0);

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

    // Distance from cone based on width
    if (Math.abs(coneWidthDifference) <= coneTargetWidthRange) // Close enough
    {
      adjustBackForward = 0;
    } else if (coneWidthDifference < 0) // Too far
    {
      adjustBackForward = 1 * DecelerationSpeed(coneWidthDifference, coneTargetWidthRange);
    } else // Too close
    {
      adjustBackForward = -1 * DecelerationSpeed(coneWidthDifference, coneTargetWidthRange);
    }
    

    // Cone left or right of robot
    if (Math.abs(coneCenterDifference) <= targetCenterXRange) // Centered
    {
      adjustLeftRight = 0;
    } else if (coneCenterDifference < 0) // Too far left
    {
      adjustLeftRight = 1 * DecelerationSpeed(coneCenterDifference, targetCenterXRange);

    } else // Too far right
    {
      adjustLeftRight = -1 * DecelerationSpeed(coneCenterDifference, targetCenterXRange);
    }

    // Distance from cube based on radius
    if (Math.abs(cubeRadiusDifference) <= cubeTargetRadiusRange) // Close enough
    {
      adjustBackForward = 0;
    } else if (cubeRadiusDifference < 0) // Too far
    {
      adjustBackForward = 1 * DecelerationSpeed(cubeRadiusDifference, cubeTargetRadiusRange);
    } else // Too close
    {
      adjustBackForward = -1 * DecelerationSpeed(cubeRadiusDifference, cubeTargetRadiusRange);
    }

    // Cube left or right of robot
    if (Math.abs(cubeCenterDifference) <= targetCenterXRange) // Centered
    {
      adjustLeftRight = 0;
    } else if (cubeCenterDifference < 0) // Too far left
    {
      adjustLeftRight = 1 * DecelerationSpeed(cubeCenterDifference, targetCenterXRange);
    } else // Too far right
    {
      adjustLeftRight = -1 * DecelerationSpeed(cubeCenterDifference, targetCenterXRange);
    }

    // Cube tag near target X
    if (Math.abs(tagCubeCenterXDifference) <= tagCubeTargetXRange) // Centered
    {
      adjustCubePlacementLeftRight = 0;
    } else if (tagCubeCenterXDifference < 0) // Too far left
    {
      adjustCubePlacementLeftRight =
          1 * DecelerationSpeed(tagCubeCenterXDifference, tagCubeTargetXRange);
    } else // Too far right
    {
      adjustCubePlacementLeftRight =
          -1 * DecelerationSpeed(tagCubeCenterXDifference, tagCubeTargetXRange);
    }

    // Distance from cube tag based on area
    if (Math.abs(tagCubeAreaDifference) <= tagCubeTargetAreaRange) // Close enough
    {
      adjustCubePlacementBackForward = 0;
    } else if (tagCubeAreaDifference < 0) // Too far
    {
      adjustCubePlacementBackForward =
          1 * DecelerationSpeed(tagCubeAreaDifference, tagCubeTargetAreaRange);
    } else // Too close
    {
      adjustCubePlacementBackForward =
          -1 * DecelerationSpeed(tagCubeAreaDifference, tagCubeTargetAreaRange);
    }

    // Cone tag near target X
    if (Math.abs(tagConeCenterXDifference) <= tagConeTargetXRange) // Centered
    {
      adjustConePlacementLeftRight = 0;
    } else if (tagConeCenterXDifference < 0) // Too far left
    {
      adjustConePlacementLeftRight =
          1 * DecelerationSpeed(tagConeCenterXDifference, tagConeTargetXRange);
    } else // Too far right
    {
      adjustConePlacementLeftRight =
          -1 * DecelerationSpeed(tagConeCenterXDifference, tagConeTargetXRange);
    }

    // Distance from cone tag based on area
    if (Math.abs(tagConeAreaDifference) <= tagConeTargetAreaRange) // Close enough
    {
      adjustConePlacementBackForward = 0;
    } else if (tagConeAreaDifference < 0) // Too far
    {
      adjustConePlacementBackForward =
          1 * DecelerationSpeed(tagConeAreaDifference, tagConeTargetAreaRange);
    } else // Too close
    {
      adjustConePlacementBackForward =
          -1 * DecelerationSpeed(tagConeAreaDifference, tagConeTargetAreaRange);
    }
  }

  private void ConePickUp() {

  }

  private void CubePickUp() {

  }

  private void ConeDropOff() {

  }

  private void CubeDropOff() {

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

  private void ChangeLEDProx(double positionDifference, double targetRange, double highestDifferenceLED) {
    double distanceFromTarget = Math.abs(positionDifference) - targetRange;
    
    if (distanceFromTarget <= highestDifferenceLED) {
      if (distanceFromTarget > 0) {
        LEDProx = (int) ((distanceFromTarget / highestDifferenceLED) * 25.0);;
      } else {
        LEDProx = 25;
      }
    } else {
      LEDProx = 0;
    }
  }

  public int GetLEDProx() {
    return LEDProx;
  }
}
