// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftLeader = new CANSparkMax(DriveConstants.kLeftLeader, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftFollower, MotorType.kBrushless);
  private final CANSparkMax m_rightLeader = new CANSparkMax(DriveConstants.kRightLeader, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kRightFollower, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final DifferentialDriveOdometry m_odometry;

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(Port.kOnboardCS0);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    m_gyro.calibrate();

    m_leftLeader.setSmartCurrentLimit(80);
    m_leftFollower.setSmartCurrentLimit(80);
    m_rightLeader.setSmartCurrentLimit(80);
    m_rightFollower.setSmartCurrentLimit(80);

    m_leftMotors.setInverted(true);

    // Circumfrence * Gear Ratio = actual distance traveled
    double factor = Math.PI * DriveConstants.kWheelDiameter * DriveConstants.kGearRatio;
    m_leftEncoder.setPositionConversionFactor(factor);
    m_rightEncoder.setPositionConversionFactor(factor);
    m_leftEncoder.setVelocityConversionFactor(factor / 60);
    m_rightEncoder.setVelocityConversionFactor(factor / 60);

    m_odometry = new DifferentialDriveOdometry(getGyroRotation());
  }

  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
  }

  public void tankDriveSet(double leftSpeed, double rightSpeed) {
    m_drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void resetEncoders() {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  public void resetOdometry() {
    zeroHeading();
    resetEncoders();
    m_odometry.resetPosition(new Pose2d(), getGyroRotation());
  }

  public void resetOdometryWithPose2d(Pose2d pose) {
    zeroHeading();
    resetEncoders();
    m_odometry.resetPosition(pose, getGyroRotation());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Putting this here so that we can simply follow a trajectory based off a given path
  // Instead of having to repeat this code over and over again in commandgroups.
  public Command followTrajectory(PathPlannerTrajectory trajectory) {
    // Reset odometry to the starting pose of the trajectory.
    resetOdometryWithPose2d(trajectory.getInitialPose());

    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        this::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.ksVolts,
            DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        this::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        this::tankDriveVolts,
        this);

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> tankDriveVolts(0, 0));
  }

  @Override
  public void periodic() {
    m_odometry.update(getGyroRotation(), -m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    Pose2d pose = getPose();
    SmartDashboard.putNumber("Left Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putNumber("odometry x", pose.getX());
    SmartDashboard.putNumber("odometry y", pose.getY());
    SmartDashboard.putNumber("odometry rotation", pose.getRotation().getDegrees());
  }
}
