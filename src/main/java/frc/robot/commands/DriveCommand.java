// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
  /** Creates a new DriveCommand. */
  DrivetrainSubsystem drivetrainSubsystem;

  SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(2);
  DoubleSupplier leftSupplier;
  DoubleSupplier rightSupplier;


  public DriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier leftSpeedSupplier, DoubleSupplier rightSpeedSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.leftSupplier = leftSpeedSupplier;
    this.rightSupplier = rightSpeedSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = leftSupplier.getAsDouble();
    double rightSpeed = rightSupplier.getAsDouble();
    drivetrainSubsystem.arcadeDrive(leftSpeed, rightSpeed);
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
