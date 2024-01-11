package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmOffCommand extends Command {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ArmSubsystem m_Arm;

  public void ArmOffCommand(ArmSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.ArmOffCommand();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
