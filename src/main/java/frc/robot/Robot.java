// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  SendableChooser<Command> GridPosChooser = new SendableChooser();
  SendableChooser<Command> TacticChooser = new SendableChooser();

  private RobotContainer m_robotContainer;

  public Trajectory m_trajectory;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    m_robotContainer = new RobotContainer();

    m_trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
            new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    SmartDashboard.putData(DriveSubsystem.m_field);

    DriveSubsystem.m_field.getObject("traj").setTrajectory(m_trajectory);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commandsa,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable fmsinfo = inst.getTable("FMSInfo");
    NetworkTableEntry isRedAlliance = fmsinfo.getEntry("IsRedAlliance");
    boolean red_alliance = isRedAlliance.getBoolean(false);
    double startingX = 0;
    double startingY = 0;
    SmartDashboard.putNumber("Auto Start Position", 1);
    int startingPos =
        Math.toIntExact(Math.round(SmartDashboard.getNumber("Auto Start Position", 1)));
    // SmartDashboard.putBoolean("isRedAlliance", red_alliance);

    // schedule the autonomous command (example)
    TacticChooser.setDefaultOption(
        "Double Shot", m_robotContainer.getAutonomousDoubleShot(red_alliance, startingPos));

    TacticChooser.addOption(
        "Shot and Power Station",
        m_robotContainer.getAutonomousShotAndPowerStation(red_alliance, startingPos));

    TacticChooser.addOption(
        "Power Station", m_robotContainer.getAutonomousPowerStation(red_alliance, startingPos));

    /*GridPosChooser.setDefaultOption("Red Grid Position 1",  m_robotContainer.getAutonomousRedGridPos1());

    GridPosChooser.addOption("Red Grid Position 2",  m_robotContainer.getAutonomousRedGridPos2());

    GridPosChooser.addOption("Blue Grid Position 1",  m_robotContainer.getAutonomousBlueGridPos1());

    GridPosChooser.addOption("Blue Grid Position 2",  m_robotContainer.getAutonomousBlueGridPos2());*/

    m_autonomousCommand = GridPosChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
