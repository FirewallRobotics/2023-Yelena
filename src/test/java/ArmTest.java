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
  ArmSubsystem m_arm = new ArmSubsystem();
  REVPhysicsSim m_sparkmaxSim;
  // public final AbsoluteEncoderSim ArmEncoder;
  public static DoubleSolenoidSim ExtendingSolenoidSim;
  public static DoubleSolenoidSim ClawSolenoidSim;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    m_sparkmaxSim = REVPhysicsSim.getInstance();
    m_sparkmaxSim.addSparkMax(
        ArmSubsystem.ArmMotor1, edu.wpi.first.math.system.plant.DCMotor.getNEO(1));
    m_sparkmaxSim.addSparkMax(
        ArmSubsystem.ArmMotor2, edu.wpi.first.math.system.plant.DCMotor.getNEO(1));
    ExtendingSolenoidSim =
        new DoubleSolenoidSim(
            PneumaticsModuleType.CTREPCM, ArmConstants.kExtSolPort1, ArmConstants.kExtSolPort2);
    ClawSolenoidSim =
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
    assertEquals(ArmConstants.kMidLength, ArmSubsystem.ArmMotor1.get());
    assertEquals(ArmConstants.kMidLength, ArmSubsystem.ArmMotor2.get());
  }

  @Test
  void armExtendHighTest() {
    m_arm.ArmExtendFarCommand();
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.ArmMotor1.get());
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.ArmMotor2.get());
  }

  @Test
  void armDefualtPositionTest() {
    m_arm.ArmRetractCommand();
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.ArmMotor1.get());
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.ArmMotor2.get());
  }
}
