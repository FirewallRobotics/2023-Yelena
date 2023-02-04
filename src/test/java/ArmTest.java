import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import edu.wpi.first.hal.HAL;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ArmTest {
  ArmSubsystem m_arm = new ArmSubsystem();
  TalonSRXSimCollection m_talonSRXextSim;
  TalonSRXSimCollection m_talonSRXliftSim;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);

    m_talonSRXextSim = new TalonSRXSimCollection(ArmSubsystem.ArmTalon1);
    m_talonSRXliftSim = new TalonSRXSimCollection(ArmSubsystem.ArmTalon2);
  }

  @AfterEach
  void shutdown() throws Exception {
    m_arm.close();
  }

  @Test
  void armExtendMedTest() {
    m_arm.ArmExtendMedCommand();
    assertEquals(ArmConstants.kMidLength, m_talonSRXextSim.getMotorOutputLeadVoltage());
  }

  @Test
  void armExtendHighTest() {
    m_arm.ArmExtendFarCommand();
    assertEquals(ArmConstants.kMaxLength, m_talonSRXextSim.getMotorOutputLeadVoltage());
  }

  @Test
  void armDefualtPositionTest() {
    m_arm.ArmRetractCommand();
    assertEquals(ArmConstants.kDefaultHeight, m_talonSRXliftSim.getMotorOutputLeadVoltage());
  }
}
