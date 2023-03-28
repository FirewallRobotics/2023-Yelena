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
  public static AbsoluteEncoder ArmEncoder;
  public static DoubleSolenoid ExtendingSolenoid;
  private SparkMaxPIDController ArmPIDController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public static double StartupPosition;

  public ArmSubsystem() {
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = .039;
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

    MasterArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    MasterArmMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    MasterArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 10);
    MasterArmMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    MinionArmMotor.follow(MasterArmMotor, false);

    ExtendingSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, ArmConstants.kExtSolPort1, ArmConstants.kExtSolPort2);

    ArmEncoder = MasterArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    ArmEncoder.setInverted(true);

    ArmPIDController = MasterArmMotor.getPIDController();
    ArmPIDController.setFeedbackDevice(ArmEncoder);
    // ArmPIDController.setPositionPIDWrappingEnabled(true);

    ArmPIDController.setP(kP);
    ArmPIDController.setI(kI);
    ArmPIDController.setD(kD);
    ArmPIDController.setIZone(kIz);
    ArmPIDController.setFF(kFF);
    ArmPIDController.setOutputRange(kMinOutput, kMaxOutput);

    ArmRetractCommand();

    StartupPosition = ArmEncoder.getPosition();
  }

  public void GravityOffset(double kdefaultheight) {
    double kMeasuredPosHorizontal =
        StartupPosition + 0.7979; // position measured when arm is horizontal (with Pheonix Tuner)
    double currentPos = ArmEncoder.getPosition();
    //double radians = currentPos - kMeasuredPosHorizontal;
    double ticks = currentPos - kMeasuredPosHorizontal;
    double radiansPerTick = 1.222/0.7979;
    double radians = ticks*radiansPerTick;
    double cosineScalar = java.lang.Math.cos(radians);
    double newHeight = StartupPosition + kdefaultheight;
    System.out.println("new height " + newHeight + " new cosineScalar " + cosineScalar + " new FF " + kFF*cosineScalar);
    SmartDashboard.putNumber("Height Setpoint", newHeight);
    SmartDashboard.putNumber("Cosine Scalar", cosineScalar);
    SmartDashboard.putNumber("Feed Forward", kFF*cosineScalar);
    ArmPIDController.setFF(kFF * cosineScalar);
    ArmPIDController.setReference(
        StartupPosition + kdefaultheight, CANSparkMax.ControlType.kPosition);
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
    GravityOffset(ArmConstants.kMidOffset);
    System.out.println("Setting arm to moderate elevation...");
  }

  public void ArmMaxHeightCommand() {
    GravityOffset(ArmConstants.kMaxOffset);
    System.out.println("Setting arm to maximum elevation...");
  }

  public void ArmGrabHeightCommand() {
    GravityOffset(ArmConstants.kGrabbingOffset);
    System.out.println("Setting arm to grabbing elevation...");
  }

  public void ArmDefaultHeightCommand() {
    GravityOffset(ArmConstants.kDefaultHeight);
    System.out.println("Returning to default elevation...");
  }

  public void ArmUpCommand() {
    MasterArmMotor.set(ArmConstants.kArmUpSpeed);
    System.out.println("Raising arm...");
  }

  public void ArmOffCommand() {
    MasterArmMotor.set(0);
    System.out.println("Stopping arm...");
  }

  public void ArmDownCommand() {
    MasterArmMotor.set(ArmConstants.kArmDownSpeed);
    System.out.println("Lowering arm...");
  }

  @Override
  public void periodic() {
    ArmEncoder.getPosition();
    /*double p = SmartDashboard.getNumber("P Gain", 0);
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
    } */

    // ArmPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);

    // SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", ArmEncoder.getPosition());
  }

  @Override
  public void close() throws Exception {
    MasterArmMotor.close();
    MinionArmMotor.close();
    ExtendingSolenoid.close();
    // ArmEncoder.close();
  }
}
