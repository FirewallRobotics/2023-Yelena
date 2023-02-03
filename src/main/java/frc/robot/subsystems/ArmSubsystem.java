package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends PIDSubsystem {
  private static WPI_TalonSRX ArmTalon1;
  private static WPI_TalonSRX ArmTalon2;
  private static Encoder encoder;

  public ArmSubsystem() {
    super(new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));
    new TalonSRX(ArmConstants.kArmMotor1);
    new TalonSRX(ArmConstants.kArmMotor2);
    Encoder encoder = new Encoder(0, 1, false, EncodingType.k2X);
  }

  public void GravityOffset(int ktargetPos) {
    int kMeasuredPosHorizontal =
        840; // position measured when arm is horizontal (with Pheonix Tuner)
    double kTicksPerDegree = 4092 / 360; // sensor is 1:1 with arm rotation
    double currentPos = ArmTalon1.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    double maxGravityFF = 0.7;
    ArmSubsystem.ArmTalon1.set(
        ControlMode.MotionMagic,
        ktargetPos,
        DemandType.ArbitraryFeedForward,
        maxGravityFF * cosineScalar);
  }

  public void ArmExtendMedCommand() {
    // will likely use pneumatics
    System.out.println("Extending to moderate length...");
  }

  public void ArmExtendFarCommand() {
    // will likely use pneumatics
    System.out.println("Extending to maximum length...");
  }

  public void ArmRetractCommand() {
    // will likely use pneumatics
    System.out.println("Returning to default length...");
  }

  public void ArmMidHeightCommand() {
    GravityOffset(ArmConstants.kMidHeight);

    System.out.println("Raising arm to moderate elevation...");
  }

  public void ArmMaxHeightCommand() {
    GravityOffset(ArmConstants.kMaxHeight);

    System.out.println("Raising arm to maximum elevation...");
  }

  public void ArmDefaultHeightCommand() {
    GravityOffset(ArmConstants.kDefaultHeight);

    System.out.println("Returning to default elevation...");
  }

  public void ClawGraspCommand() {
    // will likely use pneumatics
    System.out.println("Closing claw...");
  }

  public void ClawReleaseCommand() {
    // will likely use pneumatics
    System.out.println("Releasing claw...");
  }

  @Override
  public void periodic() {
    encoder.getDistance();
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

  @Override
  public void close() throws Exception {
    talonSRXlift.close();
    talonSRXext.close();
  }
}
