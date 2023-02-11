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
  public NetworkTableEntry ntTagCenterX = aprilTag.getEntry("Center X");
  public NetworkTableEntry ntTagCenterY = aprilTag.getEntry("Center Y");
  public NetworkTableEntry ntTagRotation = aprilTag.getEntry("rotation");
  public NetworkTableEntry ntTagArea = aprilTag.getEntry("area");

  public NetworkTableEntry ntPose = aprilTag.getEntry("Pose");
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

    double tapeCenterX = ntTapeCenterX.getDouble(0);
    double tapeCenterY = ntTapeCenterY.getDouble(0);
    double tapeArea = ntTapeArea.getDouble(0);

    double coneWidth = coneXMax - coneXMin;
    double coneCenterX = (coneXMin + coneXMax) / 2.0;

    double cameraCenterX = Constants.VisionConstants.kCameraCenterX;
    double targetCenterXRange = Constants.VisionConstants.kTargetCenterXRange;
    double coneTargetWidth = Constants.VisionConstants.kConeTargetWidth;
    double coneTargetWidthRange = Constants.VisionConstants.kConeTargetWidthRange;
    double cubeTargetRadius = Constants.VisionConstants.kCubeTargetRadius;
    double cubeTargetRadiusRange = Constants.VisionConstants.kCubeTargetRadiusRange;
    double tapeTargetX = Constants.VisionConstants.kTapeTargetX;
    double tapeTargetXRange = Constants.VisionConstants.kTapeTargetXRange;
    double tapeTargetY = Constants.VisionConstants.kTapeTargetY;
    double tapeTargetYRange = Constants.VisionConstants.kTapeTargetYRange;
    double tapeTargetArea = Constants.VisionConstants.kTapeTargetArea;
    double tapeTargetAreaRange = Constants.VisionConstants.kTapeTargetAreaRange;
    double tagTargetArea = Constants.VisionConstants.kAprilTagTargetArea;
    double tagTargetAreaRange = Constants.VisionConstants.kAprilTagTargetAreaRange;

    // Create pose from data collected from NetworkTable

    pose =
        new Pose3d(
            poseInfo[0],
            poseInfo[1],
            poseInfo[2],
            new Rotation3d(poseInfo[3], poseInfo[4], poseInfo[5]));

    // Calculating values

    // Negative if cone is left, positive if cone is right
    double tagCenterXDifference = tagCenterX - cameraCenterX;

    // Nagative if too far away, positive if too close
    double tagAreaDifference = tagArea - tagTargetArea;

    // Negative if too far away, positive if too close
    double coneWidthDifference = coneWidth - coneTargetWidth;

    // Negative if cone is left, positive if cone is right
    double coneCenterDifference = coneCenterX - cameraCenterX;

    // Negative if too far away, positive if too close
    double cubeRadiusDifference = cubeRadius - cubeTargetRadius;

    // Negative if cube is left, positive if cube is right
    double cubeCenterDifference = cubeCenterX - cameraCenterX;

    // Negative if tape is left, positive tape is right
    double tapeCenterXDifference = tapeCenterX - tapeTargetX;

    // Negative if too far away, positive if too close
    double tapeAreaDifference = tapeArea - tapeTargetArea;

    // Distance from cone based on width
    if (Math.abs(coneWidthDifference) <= coneTargetWidthRange) // Close enough
    {
      adjustBackForward = 0;
    } else if (coneWidthDifference < 0) // Too far
    {
      adjustBackForward = 1;
    } else // Too close
    {
      adjustBackForward = -1;
    }

    // Cone left or right of robot
    if (Math.abs(coneCenterDifference) <= targetCenterXRange) // Centered
    {
      adjustLeftRight = 0;
    } else if (coneCenterDifference < 0) // Too far left
    {
      adjustLeftRight = 1;
    } else // Too far right
    {
      adjustLeftRight = -1;
    }

    // Distance from cube based on radius
    if (Math.abs(cubeRadiusDifference) <= cubeTargetRadiusRange) // Close enough
    {
      adjustBackForward = 0;
    } else if (cubeRadiusDifference < 0) // Too far
    {
      adjustBackForward = 1;
    } else // Too close
    {
      adjustBackForward = -1;
    }

    // Cube left or right of robot
    if (Math.abs(cubeCenterDifference) <= targetCenterXRange) // Centered
    {
      adjustLeftRight = 0;
    } else if (cubeCenterDifference < 0) // Too far left
    {
      adjustLeftRight = 1;
    } else // Too far right
    {
      adjustLeftRight = -1;
    }

    // Tape near target X
    if (Math.abs(tapeCenterXDifference) <= tapeTargetXRange) // Centered
    {
      adjustConePlacementLeftRight = 0;
    } else if (tapeCenterX < 0) // Too far left
    {
      adjustConePlacementLeftRight = 1;
    } else // Too far right
    {
      adjustConePlacementLeftRight = -1;
    }

    /*  Tape near target Y
    if (Math.abs(tapeCenterYDifference) <= tapeTargetYRange) // Centered
    {

    }
    else if (tapeCenterY < 0) // Too low
    {

    }
    else // Too high
    {

    }
    */

    // Distance from tape based on area
    if (Math.abs(tapeAreaDifference) <= tapeTargetXRange) // Close enough
    {
      adjustConePlacementBackForward = 0;
    } else if (tapeAreaDifference < 0) // Too far
    {
      adjustConePlacementBackForward = 1;
    } else // Too close
    {
      adjustConePlacementBackForward = -1;
    }

    // Tag near target X
    if (Math.abs(tagCenterXDifference) <= targetCenterXRange) // Centered
    {
      adjustCubePlacementLeftRight = 0;
    } else if (tagCenterXDifference < 0) // Too far left
    {
      adjustCubePlacementLeftRight = 1;
    } else // Too far right
    {
      adjustCubePlacementLeftRight = -1;
    }

    // Distance from tag based on area
    if (Math.abs(tagAreaDifference) <= tagTargetAreaRange) // Close enough
    {
      adjustCubePlacementBackForward = 0;
    } else if (tapeAreaDifference < 0) // Too far
    {
      adjustCubePlacementBackForward = 1;
    } else // Too close
    {
      adjustCubePlacementBackForward = -1;
    }
  }
}
