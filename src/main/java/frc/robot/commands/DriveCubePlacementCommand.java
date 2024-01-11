// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DriveCubePlacementCommand extends Command {

  private DriveSubsystem m_drivetrain;
  private VisionSubsystem m_vision;

  private double adjustLeftRight;
  private double adjustBackForward;

  public DriveCubePlacementCommand(DriveSubsystem dt_subsystem, VisionSubsystem v_subsystem) {
    m_drivetrain = dt_subsystem;
    m_vision = v_subsystem;

    addRequirements(dt_subsystem);
    addRequirements(v_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    adjustLeftRight = m_vision.adjustCubePlacementLeftRight;
    adjustBackForward = m_vision.adjustCubePlacementBackForward;

    double speedMultiplier = Constants.AutoConstants.kAdjustSpeedMultiplier;

    if (adjustLeftRight == 0) { // Stop
      m_drivetrain.setX();
    } else if (adjustLeftRight > 0) { // Move Left
      m_drivetrain.drive(
          adjustLeftRight * speedMultiplier, 0.0, 0.0, true, true // Not sure here
          );
    } else { // Move Right
      m_drivetrain.drive(
          adjustLeftRight * speedMultiplier, 0.0, 0.0, true, true // Not sure here
          );
    }

    if (adjustBackForward == 0) { // Stop
      m_drivetrain.setX();
    } else if (adjustBackForward > 0) { // Move Forward
      m_drivetrain.drive(
          0.0, adjustBackForward * speedMultiplier, 0.0, true, true // Not sure here
          );
    } else { // Move Backward
      m_drivetrain.drive(
          0.0, adjustBackForward * speedMultiplier, 0.0, true, true // Not sure here
          );
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return adjustLeftRight == 0 && adjustBackForward == 0;
  }
}
