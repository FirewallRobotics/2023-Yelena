package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ClawSubsystem extends SubsystemBase implements AutoCloseable {
  public static DoubleSolenoid CubeSolenoid;
  public static DoubleSolenoid ConeSolenoid;

  public ClawSubsystem() {
    CubeSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, ArmConstants.kCubeSolPort1, ArmConstants.kCubeSolPort2);
    ConeSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, ArmConstants.kConeSolPort1, ArmConstants.kConeSolPort2);
  }

  public static void GrabCubeCommand() {
    CubeSolenoid.set(Value.kForward);
    System.out.println("Closing claw...");
  }

  public static void DropCubeCommand() {
    CubeSolenoid.set(Value.kReverse);
    System.out.println("Opening claw...");
  }

  public static void GrabConeCommand() {
    CubeSolenoid.set(Value.kForward);
    System.out.println("Opening claw...");
  }

  public static void DropConeCommand() {
    CubeSolenoid.set(Value.kReverse);
    System.out.println("Opening claw...");
  }

  @Override
  public void periodic() {}

  @Override
  public void close() throws Exception {
    CubeSolenoid.close();
  }
}
