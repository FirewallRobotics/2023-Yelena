package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmDefaultHeightCommand extends CommandBase {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_Arm;

  public ArmDefaultHeightCommand(ArmSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.ArmDefaultHeightCommand();
  }

  @Override
  public boolean isFinished() {
    ArmSubsystem.ArmEncoder.getPosition();
    if (m_Arm.ArmEncoder.getPosition()
        == (ArmSubsystem.StartupPosition + ArmConstants.kDefaultHeight)) return true;
    else return false;
  }
}
