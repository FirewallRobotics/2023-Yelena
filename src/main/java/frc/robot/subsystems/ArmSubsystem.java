package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends PIDSubsystem implements AutoCloseable {
  public static CANSparkMax MasterArmMotor;
  public static CANSparkMax MinionArmMotor;
  private final AbsoluteEncoder ArmEncoder;
  public static DoubleSolenoid ExtendingSolenoid;
  public static DoubleSolenoid ClawSolenoid;
  private final SparkMaxPIDController ArmPIDController;

  public ArmSubsystem() {
    super(new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD));
    MasterArmMotor = new CANSparkMax(ArmConstants.kMasterArmMotorPort, MotorType.kBrushless);
    MinionArmMotor = new CANSparkMax(ArmConstants.kMinionArmMotorPort, MotorType.kBrushless);

    MasterArmMotor.restoreFactoryDefaults();
    MinionArmMotor.restoreFactoryDefaults();

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
    ArmPIDController.setReference(ArmConstants.kMidHeight, CANSparkMax.ControlType.kPosition);
    System.out.println("Raising arm to moderate elevation...");
  }

  public void ArmMaxHeightCommand() {
    GravityOffset(ArmConstants.kMaxHeight);
    ArmPIDController.setReference(ArmConstants.kMaxHeight, CANSparkMax.ControlType.kPosition);

    System.out.println("Raising arm to maximum elevation...");
  }

  public void ArmDefaultHeightCommand() {
    GravityOffset(ArmConstants.kDefaultHeight);
    ArmPIDController.setReference(ArmConstants.kDefaultHeight, CANSparkMax.ControlType.kPosition);
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
    ArmEncoder.getPosition();
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
    MasterArmMotor.close();
    MinionArmMotor.close();
    ExtendingSolenoid.close();
    ClawSolenoid.close();
    // ArmEncoder.close();
  }
}
