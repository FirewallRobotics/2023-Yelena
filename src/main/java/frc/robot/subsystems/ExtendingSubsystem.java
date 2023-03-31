package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ExtendingSubsystem extends SubsystemBase implements AutoCloseable {
  public static DoubleSolenoid ExtSolenoid;

  public ExtendingSubsystem() {
    ExtSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, ArmConstants.kExtSolPort1, ArmConstants.kExtSolPort2);
  }

  public static void ArmExtendCommand() {
    ExtSolenoid.set(Value.kForward);
    System.out.println("Extending arm...");
  }

  public static void ArmRetractCommand() {
    ExtSolenoid.set(Value.kReverse);
    System.out.println("Retracting arm...");
  }

  @Override
  public void periodic() {}

  @Override
  public void close() throws Exception {
    ExtSolenoid.close();
  }
}
