package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMaxHeightCommand extends CommandBase {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_Arm;

  public ArmMaxHeightCommand(ArmSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.ArmMaxHeightCommand();
  }

  @Override
  public boolean isFinished() {
    ArmSubsystem.ArmEncoder.getPosition();
    if (m_Arm.ArmEncoder.getPosition() == (ArmSubsystem.StartupPosition + ArmConstants.kMaxOffset))
      return true;
    else return false;
  }
}
