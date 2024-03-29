// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutonPaths;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  
  private final XboxController driverController = new XboxController(0);
  private final SendableChooser<PathPlannerTrajectory> pathChooser = new SendableChooser<PathPlannerTrajectory>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_drivetrainSubsystem.setDefaultCommand(new DriveCommand(m_drivetrainSubsystem, () -> -driverController.getLeftY(), () -> driverController.getRightX()));
    SmartDashboard.putData("reset odometry", new InstantCommand(m_drivetrainSubsystem::resetOdometry));
    SmartDashboard.putData("zero gyro", new InstantCommand(m_drivetrainSubsystem::zeroHeading));
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    pathChooser.setDefaultOption("Bounce", AutonPaths.Bounce);
    pathChooser.addOption("Funny", AutonPaths.Funny);

    SmartDashboard.putData("Path Chooser", pathChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Follow the test bounce path
    return m_drivetrainSubsystem.followTrajectory(pathChooser.getSelected());
  }
}
