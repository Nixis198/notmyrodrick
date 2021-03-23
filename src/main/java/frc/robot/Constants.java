// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

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
        public static final int kLeftMotor1Port = 1;
        public static final int kLeftMotor2Port = 2;
        public static final int kRightMotor1Port = 3;
        public static final int kRightMotor2Port = 4;
        public static final int kEncoderCPR = 2048;
        public static final double kWheelDiameterInches = 6;
        public static final double kTrackWidthInches = 27.16535;
        public static final double kGearRatio = (50 / 14) * (48 / 16);
        public static final double kWheelCircumferenceMeters = Units.inchesToMeters(kWheelDiameterInches) * Math.PI;
        public static final double kMetersPerTick = Units.inchesToMeters(kWheelCircumferenceMeters) / (kEncoderCPR * kGearRatio);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 1;
        public static final int kOperatorControllerPort = 2;
        public static final double kSlowDriveLimit = 0.5;
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorPort = 7;
    }

    public static final class PneumaticsConstants {
        public static final int kGateSolenoidPort = 0;
    }

    public static final class ClimbConstants {
        public static final int kWinchMotor = 5;
        public static final int kTelescopeMotor = 6;
        public static final int kEncoderCPR = 2048;
        public static final double kTelescopeSpeedUp = 0.3;
        public static final double kTelescopeSpeedDown = -0.1;
        public static final double kRotationsToTop = 11;
    }

    public static final class TrajectoryConstants {
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;
        public static final double kPDriveVel = 8.5;
        public static final double kTrackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
