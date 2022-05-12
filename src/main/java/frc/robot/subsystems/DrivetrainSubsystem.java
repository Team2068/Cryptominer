// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase {

  private final CANSparkMax m_leftLeader = new CANSparkMax(DriveConstants.kLeftLeader, MotorType.kBrushless);
  private final CANSparkMax m_leftFollower = new CANSparkMax(DriveConstants.kLeftFollower, MotorType.kBrushless);
  private final CANSparkMax m_rightLeader = new CANSparkMax(DriveConstants.kRightLeader, MotorType.kBrushless);
  private final CANSparkMax m_rightFollower = new CANSparkMax(DriveConstants.kLeftFollower, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightLeader.getEncoder();

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(23.5));
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(getGyroHeading());
  private Pose2d m_pose;

  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro(Port.kOnboardCS0);

  /** Creates a new DrivetrainSubsystem. */
  public DrivetrainSubsystem() {
    m_leftLeader.setSmartCurrentLimit(30);
    m_leftFollower.setSmartCurrentLimit(30);
    m_rightLeader.setSmartCurrentLimit(30);
    m_rightFollower.setSmartCurrentLimit(30);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    double factor = Math.PI * Units.inchesToMeters(5) * 7.75; // Gear ratio 7.75:1
    m_leftEncoder.setPositionConversionFactor(factor);
    m_rightEncoder.setPositionConversionFactor(factor);
    m_leftEncoder.setVelocityConversionFactor(factor / 60);
    m_rightEncoder.setVelocityConversionFactor(factor / 60);

    m_pose = new Pose2d();
  }

  public Rotation2d getGyroHeading() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(360, m_gyro.getAngle()));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    // TODO: implement!
  }

  @Override
  public void periodic() {
    m_pose = m_odometry.update(getGyroHeading(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Position", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("Right Position", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("Left Velocity", m_leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Velocity", m_rightEncoder.getVelocity());
  }
}
