package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase implements AutoCloseable {
  public static CANSparkMax MasterArmMotor;
  public static CANSparkMax MinionArmMotor;
  private final AbsoluteEncoder ArmEncoder;
  public static DoubleSolenoid ExtendingSolenoid;
  public static DoubleSolenoid ClawSolenoid;
  private SparkMaxPIDController ArmPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public ArmSubsystem() {
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0.3;
    kMaxOutput = 1;
    kMinOutput = -1;

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

    MasterArmMotor = new CANSparkMax(ArmConstants.kMasterArmMotorPort, MotorType.kBrushless);
    MinionArmMotor = new CANSparkMax(ArmConstants.kMinionArmMotorPort, MotorType.kBrushless);

    MasterArmMotor.restoreFactoryDefaults();
    MinionArmMotor.restoreFactoryDefaults();

    MasterArmMotor.setInverted(true);
    MinionArmMotor.follow(MasterArmMotor);

    ExtendingSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, ArmConstants.kExtSolPort1, ArmConstants.kExtSolPort2);
    ClawSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, ArmConstants.kClawSolPort1, ArmConstants.kClawSolPort2);

    ArmEncoder = MasterArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    ArmEncoder.setInverted(true);

    ArmPIDController = MasterArmMotor.getPIDController();
    ArmPIDController.setFeedbackDevice(ArmEncoder);

    ArmPIDController.setP(kP);
    ArmPIDController.setI(kI);
    ArmPIDController.setD(kD);
    ArmPIDController.setIZone(kIz);
    ArmPIDController.setFF(kFF);
    ArmPIDController.setOutputRange(kMinOutput, kMaxOutput);
  }

  public void GravityOffset(int ktargetPos) {
    int kMeasuredPosHorizontal =
        840; // position measured when arm is horizontal (with Pheonix Tuner)
    double kTicksPerDegree = 4092 / 360; // sensor is 1:1 with arm rotation
    double currentPos = ArmEncoder.getPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    double maxGravityFF = 0.7;
    ArmPIDController.setReference(radians, CANSparkMax.ControlType.kPosition);
  }

  public static void ArmExtendCommand() {
    ExtendingSolenoid.set(Value.kForward);
    System.out.println("Extending arm...");
  }

  public static void ArmRetractCommand() {
    ExtendingSolenoid.set(Value.kReverse);
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

  public void ClawGrabConeCommand() {
    ClawSolenoid.set(Value.kForward);
    System.out.println("Grabbing cone...");
  }

  public void ClawReleaseCommand() {
    ClawSolenoid.set(Value.kReverse);
    System.out.println("Releasing claw...");
  }

  public void TestArmMotorsCommand() {
    MasterArmMotor.set(.5);
    System.out.println("TestingArmMotors");
  }

  @Override
  public void periodic() {
    ArmEncoder.getPosition();
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if ((p != kP)) {
      ArmPIDController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      ArmPIDController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      ArmPIDController.setD(d);
      kD = d;
    }
    if ((iz != kIz)) {
      ArmPIDController.setIZone(iz);
      kIz = iz;
    }
    if ((ff != kFF)) {
      ArmPIDController.setFF(ff);
      kFF = ff;
    }
    if ((max != kMaxOutput) || (min != kMinOutput)) {
      ArmPIDController.setOutputRange(min, max);
      kMinOutput = min;
      kMaxOutput = max;
    }

    ArmPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);

    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", ArmEncoder.getPosition());
  }

  @Override
  public void close() throws Exception {
    MasterArmMotor.close();
    MinionArmMotor.close();
    ExtendingSolenoid.close();
    ClawSolenoid.close();
    // ArmEncoder.close();
  }
}
