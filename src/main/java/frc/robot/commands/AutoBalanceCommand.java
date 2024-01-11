package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class AutoBalanceCommand extends Command {
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
    ;
    double xAngle = m_drive.m_gyro.getXComplementaryAngle();
    double yAngle = m_drive.m_gyro.getYComplementaryAngle();

    double xVeloc;
    double yVeloc;

    if (Math.abs(xAngle) <= gyroAngleRange) // Balanced
    {
      xVeloc = 0.0;
    } else if (xAngle < 0) // Too far back
    {
      xVeloc = 1.0 * DecelerationSpeed(xAngle, gyroAngleRange) * balanceSpeedMultiplier;
    } else // Too far forward
    {
      xVeloc = -1.0 * DecelerationSpeed(xAngle, gyroAngleRange) * balanceSpeedMultiplier;
    }

    if (Math.abs(yAngle) <= gyroAngleRange) // Balanced
    {
      yVeloc = 0.0;
    } else if (yAngle < 0) // Too far back
    {
      yVeloc = 1.0 * DecelerationSpeed(yAngle, gyroAngleRange) * balanceSpeedMultiplier;
    } else // Too far forward
    {
      yVeloc = -1.0 * DecelerationSpeed(yAngle, gyroAngleRange) * balanceSpeedMultiplier;
    }

    if (xVeloc == 0.0 && yVeloc == 0.0) {
      m_drive.setX();
    } else {
      m_drive.drive(
          xVeloc, yVeloc, 0.0, true, true // Not sure here
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
