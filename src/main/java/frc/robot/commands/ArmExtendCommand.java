package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExtendingSubsystem;

public class ArmExtendCommand extends CommandBase {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtendingSubsystem m_Ext;

  public ArmExtendCommand(ExtendingSubsystem subsystem) {
    m_Ext = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Ext.ArmExtendCommand();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
