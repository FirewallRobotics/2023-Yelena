package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ClawSubsystem extends SubsystemBase implements AutoCloseable {
  public static DoubleSolenoid ClawSolenoid;

  public ClawSubsystem() {
    ClawSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, ArmConstants.kClawSolPort1, ArmConstants.kClawSolPort2);
  }

  public static void ClawGrabCommand() {
    ClawSolenoid.set(Value.kForward);
    System.out.println("Closing claw...");
  }

  public static void ClawReleaseCommand() {
    ClawSolenoid.set(Value.kReverse);
    System.out.println("Opening claw...");
  }

  @Override
  public void periodic() {}

  @Override
  public void close() throws Exception {
    ClawSolenoid.close();
  }
}
