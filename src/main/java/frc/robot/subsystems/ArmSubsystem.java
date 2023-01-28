package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends PIDSubsystem {
  private WPI_TalonSRX talonSRX1;
  private WPI_TalonSRX talonSRX2;

  public ArmSubsystem() {
    super(new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));
    new TalonSRX(ArmConstants.kArmMotor1);
    new TalonSRX(ArmConstants.kArmMotor2);
  }

  public void ArmExtendMedCommand() {

    System.out.println("Extending to moderate length...");
  }

  public void ArmExtendFarCommand() {

    System.out.println("Extending to maximum length...");
  }

  public void ArmRetractCommand() {

    System.out.println("Returning to default length...");
  }

  public void ArmMidHeightCommand() {

    System.out.println("Raising arm to moderate elevation...");
  }

  public void ArmMaxHeightCommand() {

    System.out.println("Raising arm to maximum elevation...");
  }

  public void ArmDefaultHeightCommand() {

    System.out.println("Returning to default position...");
  }

  public void ClawGraspCommand() {

    System.out.println("Closing claw...");
  }

  public void ClawReleaseCommand() {

    System.out.println("Releasing claw...");
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub

  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }
}
