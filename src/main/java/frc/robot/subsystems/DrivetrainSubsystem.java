// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kLeftFollower, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
  private final DifferentialDriveOdometry m_odometry;

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(Port.kOnboardCS0);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    m_gyro.calibrate();

    m_leftLeader.setSmartCurrentLimit(30);
    m_leftFollower.setSmartCurrentLimit(30);
    m_rightLeader.setSmartCurrentLimit(30);
    m_rightFollower.setSmartCurrentLimit(30);

    m_rightLeader.setInverted(true);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    // Circumfrence * Gear Ratio = actual distance traveled
    double factor = Math.PI * Units.inchesToMeters(5) * 7.75 / m_leftEncoder.getCountsPerRevolution(); // Gear ratio 7.75:1
    m_leftEncoder.setPositionConversionFactor(factor);
    m_rightEncoder.setPositionConversionFactor(factor);
    m_leftEncoder.setVelocityConversionFactor(factor / 60);
    m_rightEncoder.setVelocityConversionFactor(factor / 60);

    m_odometry = new DifferentialDriveOdometry(getGyroRotation());
  }

  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(360, m_gyro.getAngle()));
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void arcadeDrive(double speed, double rotation) {
    m_drive.arcadeDrive(speed, rotation);
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
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  // Putting this here so that we can simply follow a trajectory based off a given path
  // Instead of having to repeat this code over and over again in commandgroups.
  public Command followTrajectory(Trajectory trajectory) {
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

    // Reset odometry to the starting pose of the trajectory.
    resetOdometryWithPose2d(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> tankDriveVolts(0, 0));
  }

  @Override
  public void periodic() {
    m_odometry.update(getGyroRotation(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getVelocity());
  }
}
