package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ClawGrabCommand extends Command {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClawSubsystem m_Claw;

  public ClawGrabCommand(ClawSubsystem subsystem) {
    m_Claw = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Claw.ClawGrabCommand();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
