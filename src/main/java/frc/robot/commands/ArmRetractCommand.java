package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExtendingSubsystem;

public class ArmRetractCommand extends Command {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtendingSubsystem m_Ext;

  public ArmRetractCommand(ExtendingSubsystem subsystem) {
    m_Ext = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Ext.ArmRetractCommand();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
