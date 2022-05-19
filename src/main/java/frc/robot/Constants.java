// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftLeader = 5;
        public static final int kLeftFollower = 3;
        public static final int kRightLeader = 4;
        public static final int kRightFollower = 2;

        public static final double kTrackwidthMeters = Units.inchesToMeters(23.5);
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kPDriveVel = 2.4122;
        public static final double ksVolts = 0.16737;
        public static final double kvVoltSecondsPerMeter = 2.454;
        public static final double kaVoltSecondsSquaredPerMeter = 0.34951;

        public static final double kGearRatio = 1 / 7.75;
        public static final double kWheelDiameter = Units.inchesToMeters(5);
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }

    public static final class AutonPaths {
        public static final PathPlannerTrajectory Bounce = PathPlanner.loadPath("Bounce", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
        public static final PathPlannerTrajectory Funny = PathPlanner.loadPath("Funny", AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    }
}
