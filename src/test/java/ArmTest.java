package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import frc.robot.Constants.ArmConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ArmTest {
  ArmSubsystem m_arm;
  TalonSRXSimCollection m_talonSRXextSim;
  TalonSRXSimCollection m_talonSRXliftSim;

  @BeforeEach
  void setup() {
    m_arm = new ArmSubsystem();
    m_talonSRXextSim = new TalonSRXSimCollection(m_arm.talonSRXext);
    m_talonSRXliftSim = new TalonSRXSimCollection(m_arm.talonSRXlift);
  }

  @AfterEach
  void shutdown() throws Exception {
    m_arm.close();
  }

  @Test
  void armExtendMedTest() {
    m_arm.ArmExtendMedCommand();
    assertEquals(ArmConstants.kMidPosition, m_talonSRXextSim.getMotorOutputLeadVoltage());
  }

  @Test
  void armExtendHighTest() {
    m_arm.ArmExtendHighCommand();
    assertEquals(ArmConstants.kHighPosition, m_talonSRXextSim.getMotorOutputLeadVoltage());
  }

  @Test
  void armDefualtPositionTest() {
    m_arm.ArmDefaultPositionCommand();
    assertEquals(ArmConstants.kDefaultPosition, m_talonSRXliftSim.getMotorOutputLeadVoltage());
  }
}
