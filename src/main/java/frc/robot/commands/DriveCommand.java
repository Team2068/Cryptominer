// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  DrivetrainSubsystem drivetrainSubsystem;

  private double xSpeed;
  private double rotationSpeed;

  SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);

  public DriveCommand(DrivetrainSubsystem drivetrainSubsystem, double xSpeed, double rotationSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.xSpeed = xSpeed;
    this.rotationSpeed = rotationSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xSpeedLimiter.calculate(xSpeed); // add ramp rate to driver control
    drivetrainSubsystem.arcadeDrive(xSpeed, rotationSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.tankDriveVolts(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
