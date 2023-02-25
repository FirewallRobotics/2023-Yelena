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
  DoubleSolenoidSim m_extendingDoubleSolenoidSim;
  DoubleSolenoidSim m_clawDoubleSolenoidSim;

  @BeforeEach
  void setup() {
    assert HAL.initialize(500, 0);
    m_sparkmaxSim = REVPhysicsSim.getInstance();
    m_sparkmaxSim.addSparkMax(
        ArmSubsystem.MasterArmMotor, edu.wpi.first.math.system.plant.DCMotor.getNEO(1));
    m_sparkmaxSim.addSparkMax(
        ArmSubsystem.MinionArmMotor, edu.wpi.first.math.system.plant.DCMotor.getNEO(2));

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
  void armExtendHighTest() {
    m_arm.ArmExtendCommand();
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.MasterArmMotor.get());
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.MinionArmMotor.get());
  }

  @Test
  void armDefualtPositionTest() {
    m_arm.ArmRetractCommand();
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.MasterArmMotor.get());
    assertEquals(ArmConstants.kMaxLength, ArmSubsystem.MinionArmMotor.get());
  }
}
