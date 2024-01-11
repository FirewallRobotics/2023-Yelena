package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMidHeightCommand extends Command {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_Arm;

  public ArmMidHeightCommand(ArmSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.ArmMidHeightCommand();
  }

  @Override
  public boolean isFinished() {
    ArmSubsystem.ArmEncoder.getPosition();
    if (m_Arm.ArmEncoder.getPosition() == ArmConstants.kMidHeight) return true;
    else return false;
  }
}
