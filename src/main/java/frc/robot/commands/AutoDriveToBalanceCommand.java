// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoDriveToBalanceCommand extends CommandBase {

  private DriveSubsystem m_drive;
  private VisionSubsystem m_vision;
  private boolean isFinished = false;

  private double driveToBalanceSpeed = Constants.DriveConstants.kDriveToBalanceSpeedMultiplier;
  private double driveGyroAngleRange = Constants.DriveConstants.kDriveToBalanceGyroAngleRange;

  public AutoDriveToBalanceCommand(DriveSubsystem d_subsystem, VisionSubsystem v_subsystem) {
    m_drive = d_subsystem;
    m_vision = v_subsystem;
    addRequirements(d_subsystem);
    addRequirements(v_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_drive.m_gyro.getXComplementaryAngle();

    if (Math.abs(angle) <= driveGyroAngleRange) {
      m_drive.drive(-1 * driveToBalanceSpeed, 0, 0, isFinished, isFinished);
    } else {
      isFinished = true;
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
