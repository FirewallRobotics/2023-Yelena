// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DriveDpadSneakCommand extends Command {
  /** Creates a new DriveDpadSneakCommand. */
  private DriveSubsystem m_drivetrain;

  private XboxController m_robotController;

  public DriveDpadSneakCommand(DriveSubsystem dt_subsystem, XboxController xb_controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = dt_subsystem;
    m_robotController = xb_controller;

    addRequirements(dt_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double sneakSpeed = Constants.DriveConstants.kSneakSpeed;

    if (m_robotController.getPOV() == 0) {
      m_drivetrain.drive(sneakSpeed, 0, 0, true, true);
    } else if (m_robotController.getPOV() == 45) {
      m_drivetrain.drive(sneakSpeed, -sneakSpeed, 0, true, true);
    } else if (m_robotController.getPOV() == 90) {
      m_drivetrain.drive(0, -sneakSpeed, 0, true, true);
    } else if (m_robotController.getPOV() == 135) {
      m_drivetrain.drive(-sneakSpeed, -sneakSpeed, 0, true, true);
    } else if (m_robotController.getPOV() == 180) {
      m_drivetrain.drive(-sneakSpeed, 0, 0, true, true);
    } else if (m_robotController.getPOV() == 225) {
      m_drivetrain.drive(-sneakSpeed, sneakSpeed, 0, true, true);
    } else if (m_robotController.getPOV() == 270) {
      m_drivetrain.drive(0, sneakSpeed, 0, true, true);
    } else if (m_robotController.getPOV() == 315) {
      m_drivetrain.drive(sneakSpeed, sneakSpeed, 0, true, true);
    } else {
      m_drivetrain.drive(0, 0, 0, true, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
