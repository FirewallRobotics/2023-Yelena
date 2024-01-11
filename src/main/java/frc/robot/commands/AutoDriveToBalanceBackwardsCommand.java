// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoDriveToBalanceBackwardsCommand extends Command {

  private DriveSubsystem m_drive;
  private boolean isFinished = false;

  private double driveToBalanceSpeed = Constants.AutoConstants.kDriveToBalanceSpeedMultiplier;
  private double driveGyroAngleRange = Constants.AutoConstants.kDriveToBalanceGyroAngleRange;
  // private double driveToBalanceDelay = Constants.AutoConstants.kDriveToBalanceDelay;
  private boolean onBalance = false;
  // private int timer = 0;

  public AutoDriveToBalanceBackwardsCommand(DriveSubsystem d_subsystem) {
    m_drive = d_subsystem;
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xAngle = m_drive.m_gyro.getXComplementaryAngle();

    if (onBalance) {
      if (Math.abs(xAngle) > driveGyroAngleRange) {
        m_drive.drive(1 * driveToBalanceSpeed, 0, 0, true, true);
      } else {
        isFinished = true;
      }
    } else {
      if (Math.abs(xAngle) <= driveGyroAngleRange) {
        m_drive.drive(1 * driveToBalanceSpeed, 0, 0, true, true);
      } else {
        onBalance = true;
      }
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
