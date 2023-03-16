package frc.robot.commands;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoBalanceCommand extends CommandBase {
  private DriveSubsystem m_drive;
  private boolean isFinished = false;

  private double gyroAngleRange = Constants.DriveConstants.kGyroAngleRange;
  private double balanceSpeedMultiplier = Constants.DriveConstants.kBalanceSpeedMultiplier;
  private double balanceDecelerationDistance =
      Constants.DriveConstants.kBalanceDecelerationDistance;

  public AutoBalanceCommand(DriveSubsystem d_subsystem) {
    m_drive = d_subsystem;
    
    addRequirements(d_subsystem);

  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub

    m_drive.m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kX);
    double angle = m_drive.m_gyro.getAngle();
    m_drive.m_gyro.setYawAxis(ADIS16470_IMU.IMUAxis.kY);


    if (Math.abs(angle) <= gyroAngleRange) // Balanced
    {
      m_drive.drive(
          0.0, 0.0, 0.0, true, true // Not sure here
          );
    } else if (angle < 0) // Too far back
    {
      m_drive.drive(
          1.0 * DecelerationSpeed(angle, gyroAngleRange) * balanceSpeedMultiplier,
          0.0,
          0.0,
          true,
          true // Not sure here
          );
    } else // Too far forward
    {
      m_drive.drive(
          -1.0 * DecelerationSpeed(angle, gyroAngleRange) * balanceSpeedMultiplier,
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
