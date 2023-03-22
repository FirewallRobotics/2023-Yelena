// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoDriveToBalanceCommand extends CommandBase {

  private DriveSubsystem m_drive;
  private boolean isFinished = false;

  private double driveToBalanceSpeed = Constants.DriveConstants.kDriveToBalanceSpeedMultiplier;
  private double driveGyroAngleRange = Constants.DriveConstants.kDriveToBalanceGyroAngleRange;

  public AutoDriveToBalanceCommand(DriveSubsystem d_subsystem) {
    m_drive = d_subsystem;
    addRequirements(d_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kX);
    double xAngle = m_drive.m_gyro.getAngle();
    m_drive.m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);

    if (Math.abs(xAngle) <= driveGyroAngleRange) {
      m_drive.drive(-1 * driveToBalanceSpeed, 0, 0, true, true);
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
