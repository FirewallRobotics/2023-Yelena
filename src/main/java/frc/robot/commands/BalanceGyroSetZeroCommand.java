// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class BalanceGyroSetZeroCommand extends Command {
  /** Creates a new AutoBalanceZeroGyro. */
  private DriveSubsystem m_drivetrain;

  private boolean isFinished = false;

  public BalanceGyroSetZeroCommand(DriveSubsystem dt_subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrain = dt_subsystem;
    addRequirements(dt_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Resetting gyro, zero heading");
    m_drivetrain.m_gyro.reset();
    m_drivetrain.zeroHeading();
    isFinished = true;
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
