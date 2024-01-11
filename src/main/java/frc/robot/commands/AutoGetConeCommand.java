package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class AutoGetConeCommand extends Command {
  private DriveSubsystem m_drive;
  private VisionSubsystem m_vision;
  private boolean isFinished = false;

  public AutoGetConeCommand(DriveSubsystem d_subsystem, VisionSubsystem v_subsystem) {
    m_drive = d_subsystem;
    m_vision = v_subsystem;
    addRequirements(d_subsystem);
    addRequirements(v_subsystem);
  }
}
