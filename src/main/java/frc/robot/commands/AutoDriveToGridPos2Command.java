package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class AutoDriveToGridPos2Command extends CommandBase {

  private DriveSubsystem m_drive;
  private VisionSubsystem m_vision;
  private boolean isFinished = false;

  public AutoDriveToGridPos2Command(DriveSubsystem d_subsystem, VisionSubsystem v_subsystem) {
    m_drive = d_subsystem;
    m_vision = v_subsystem;
    addRequirements(d_subsystem);
    addRequirements(v_subsystem);
  }

  @Override
  public void execute() {
    m_drive.drive(
        0.0, 0.0, 180.0, true, true // Not sure here
        );
  }
}
