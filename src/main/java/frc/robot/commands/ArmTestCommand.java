package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmTestCommand extends CommandBase {

  @Override
  public void initialize() {}

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_Arm;

  public ArmTestCommand(ArmSubsystem subsystem) {
    m_Arm = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void execute() {
    m_Arm.ArmTestCommand();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interupted) {
    m_Arm.ArmTestOffCommand();
  }
}
