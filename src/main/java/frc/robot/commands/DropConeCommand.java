package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class DropConeCommand extends CommandBase {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClawSubsystem m_Claw;

  public DropConeCommand(ClawSubsystem subsystem) {
    m_Claw = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Claw.DropConeCommand();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
