import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.hal.HAL;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ArmTest {
  ArmSubsystem m_arm = new ArmSubsystem();
  REVPhysicsSim m_sparkmaxSim;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    m_sparkmaxSim = REVPhysicsSim.getInstance();
    m_sparkmaxSim.addSparkMax(
        ArmSubsystem.ArmMotor1, edu.wpi.first.math.system.plant.DCMotor.getNEO(1));
    m_sparkmaxSim.addSparkMax(
        ArmSubsystem.ArmMotor2, edu.wpi.first.math.system.plant.DCMotor.getNEO(2));
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
