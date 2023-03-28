// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import java.util.List;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // SendableChooser<Command> GridPosChooser = new SendableChooser();
  SendableChooser<Command> TacticChooser = new SendableChooser();

  private RobotContainer m_robotContainer;
  public NetworkTableEntry robotState;
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
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);
    m_trajectory =
        /*.generateTrajectory(
        List.of(
                new Pose2d(AutoConstants.startingX3 + .4, AutoConstants.startingY3, new Rotation2d(Math.PI/2)),
                new Pose2d(AutoConstants.startingX3 + .4, AutoConstants.startingY3 + .375, new Rotation2d(Math.PI/2)),
                new Pose2d(AutoConstants.startingX3 + .4, AutoConstants.startingY3 + .75, new Rotation2d (Math.PI)),
                new Pose2d(AutoConstants.startingX3 - 1.7, AutoConstants.startingY3 + .75, new Rotation2d (Math.PI)),
                new Pose2d(AutoConstants.startingX3 - 3.4, AutoConstants.startingY3 + .75, new Rotation2d (Math.PI))),
                config);*/

        // Start at the origin facing the +X direction
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(AutoConstants.startingX4 - .4, AutoConstants.startingY4, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(AutoConstants.startingX4 + .8, AutoConstants.startingY4)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(AutoConstants.startingX4 + 1.6, AutoConstants.startingY4, new Rotation2d(0)),
            config);

    // SmartDashboard.putData(DriveSubsystem.m_field);

    // DriveSubsystem.m_field.getObject("traj").setTrajectory(m_trajectory);

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
    SmartDashboard.putNumber("Auto Start Position", 1);
    // int startingPos =
    Math.toIntExact(Math.round(SmartDashboard.getNumber("Auto Start Position", 1)));
    // SmartDashboard.putBoolean("isRedAlliance", red_alliance);

    // schedule the autonomous command (example)
    /*TacticChooser.setDefaultOption(
    "Double Shot", m_robotContainer.getAutonomousDoubleShot(red_alliance, startingPos));*/
    TacticChooser.setDefaultOption(
        "Power Station Pos 1", m_robotContainer.getAutonomousPowerStation(red_alliance, 1));

    TacticChooser.addOption(
        "Leave Community Pos 1", m_robotContainer.getAutonomousLeaveCommunity(red_alliance, 1));

    TacticChooser.addOption(
        "Single Score Pos 1", m_robotContainer.getAutonomousScore(red_alliance, 1));

    TacticChooser.addOption(
        "Single Score Pos 2", m_robotContainer.getAutonomousScore(red_alliance, 2));

    TacticChooser.addOption(
        "Single Score Pos 3", m_robotContainer.getAutonomousScore(red_alliance, 3));

    TacticChooser.addOption(
        "Power Station and Score Pos 1",
        m_robotContainer.getAutonomousScoreAndPowerStation(red_alliance, 1));

    /*TacticChooser.addOption(
    "Double Score", m_robotContainer.getAutonomousScore(red_alliance, startingPos));*/

    SmartDashboard.putData("Autonomous Mode", TacticChooser);

    /*GridPosChooser.setDefaultOption("Red Grid Position 1",  m_robotContainer.getAutonomousRedGridPos1());

    GridPosChooser.addOption("Red Grid Position 2",  m_robotContainer.getAutonomousRedGridPos2());

    GridPosChooser.addOption("Blue Grid Position 1",  m_robotContainer.getAutonomousBlueGridPos1());

    GridPosChooser.addOption("Blue Grid Position 2",  m_robotContainer.getAutonomousBlueGridPos2());*/
    CameraServer.startAutomaticCapture();
    robotState = inst.getEntry("RobotState");
    robotState.setString("init");
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
  public void disabledInit() {
    robotState.setString("disabled");
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = TacticChooser.getSelected();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      System.out.println("Scheduled");
    }
    robotState.setString("autonomous");
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
      System.out.println("Cancel");
    }
    robotState.setString("teleop");
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    robotState.setString("test");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
