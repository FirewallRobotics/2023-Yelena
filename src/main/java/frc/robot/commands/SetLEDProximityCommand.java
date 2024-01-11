// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;

public class SetLEDProximityCommand extends Command {

  private VisionSubsystem m_vision;
  private LEDSubsystem m_LED;

  public SetLEDProximityCommand(VisionSubsystem v_subsystem, LEDSubsystem led_subsystem) {
    m_vision = v_subsystem;
    m_LED = led_subsystem;

    addRequirements(v_subsystem);
    addRequirements(led_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_LED.RunDefaultLED(m_vision.GetLEDProx());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
