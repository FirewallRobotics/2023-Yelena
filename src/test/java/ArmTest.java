import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ArmTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  ArmSubsystem m_arm = new ArmSubsystem();
  REVPhysicsSim m_sparkmaxSim;
  DoubleSolenoidSim m_extendingDoubleSolenoidSim;
  DoubleSolenoidSim m_clawDoubleSolenoidSim;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    m_sparkmaxSim = REVPhysicsSim.getInstance();
    m_sparkmaxSim.addSparkMax(
        ArmSubsystem.MasterArmMotor, edu.wpi.first.math.system.plant.DCMotor.getNEO(1));
    m_sparkmaxSim.addSparkMax(
        ArmSubsystem.MinionArmMotor, edu.wpi.first.math.system.plant.DCMotor.getNEO(1));

    m_extendingDoubleSolenoidSim =
        new DoubleSolenoidSim(
            PneumaticsModuleType.CTREPCM, ArmConstants.kExtSolPort1, ArmConstants.kExtSolPort2);
    m_clawDoubleSolenoidSim =
        new DoubleSolenoidSim(
            PneumaticsModuleType.CTREPCM, ArmConstants.kClawSolPort1, ArmConstants.kClawSolPort2);
  }

  @AfterEach
  void shutdown() throws Exception {
    m_arm.close();
  }

  @Test
  void armExtendMedTest() {
    m_arm.ArmExtendMedCommand();
    assertEquals(ArmConstants.kMidLength, ArmSubsystem.MasterArmMotor.get(), DELTA);
    assertEquals(ArmConstants.kMidLength, ArmSubsystem.MinionArmMotor.get(), DELTA);
  }

  @Test
  void armExtendHighTest() {
    m_arm.ArmExtendFarCommand();
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.MasterArmMotor.get(), DELTA);
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.MinionArmMotor.get(), DELTA);
  }

  @Test
  void armDefualtPositionTest() {
    m_arm.ArmRetractCommand();
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.MasterArmMotor.get(), DELTA);
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.MinionArmMotor.get(), DELTA);
  }

  @Test
  void ArmMidHeightCommandTest() {
    m_arm.ArmMidHeightCommand();
    assertEquals(ArmConstants.kMidHeight, m_arm.ArmEncoder.getPosition(), DELTA);
    System.out.println("Mid height " + m_arm.ArmEncoder.getPosition());
  }

  @Test
  void ArmMaxHeightCommandTest() {
    m_arm.ArmMaxHeightCommand();
    assertEquals(ArmConstants.kMaxHeight, m_arm.ArmEncoder.getPosition(), DELTA);
    System.out.println("Max height " + m_arm.ArmEncoder.getPosition());
  }

  @Test
  void ArmDefaultHeightCommandTest() {
    m_arm.ArmDefaultHeightCommand();
    assertEquals(ArmConstants.kDefaultHeight, m_arm.ArmEncoder.getPosition(), DELTA);
    System.out.println("Default Height " + m_arm.ArmEncoder.getPosition());
  }
}
