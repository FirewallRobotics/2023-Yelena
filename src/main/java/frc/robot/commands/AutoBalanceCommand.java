package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoBalanceCommand extends CommandBase {
  private DriveSubsystem m_drive;
  private VisionSubsystem m_vision;
  private boolean isFinished = false;

  private double gyroAngleRange = Constants.DriveConstants.kGyroAngleRange;
  private double balanceSpeedMultiplier = Constants.DriveConstants.kBalanceSpeedMultiplier;
  private double balanceDecelerationDistance =
      Constants.DriveConstants.kBalanceDecelerationDistance;

  public AutoBalanceCommand(DriveSubsystem d_subsystem, VisionSubsystem v_subsystem) {
    m_drive = d_subsystem;
    m_vision = v_subsystem;
    addRequirements(d_subsystem);
    addRequirements(v_subsystem);
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub

    double angle = m_drive.m_gyro.getYComplementaryAngle();

    if (Math.abs(angle) <= gyroAngleRange) // Balanced
    {
      m_drive.drive(
          0.0, 0.0, 0.0, true, true // Not sure here
          );
    } else if (angle < 0) // Too far back
    {
      m_drive.drive(
          1.0 * DecelerationSpeed(angle, angle) * balanceSpeedMultiplier,
          0.0,
          0.0,
          true,
          true // Not sure here
          );
    } else // Too far forward
    {
      m_drive.drive(
          -1.0 * DecelerationSpeed(angle, angle) * balanceSpeedMultiplier,
          0.0,
          0.0,
          true,
          true // Not sure here
          );
    }
  }

  private double DecelerationSpeed(double positionDifference, double targetRange) {
    double distanceFromTarget = Math.abs(positionDifference) - targetRange;
    double speed = (distanceFromTarget / balanceDecelerationDistance) * 9.0 / 10.0 + 0.1;

    if (speed < 1) // Max value for speed is 1
    {
      return speed;
    } else {
      return 1.0;
    }
  }
}
