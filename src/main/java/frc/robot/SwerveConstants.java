package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerveUtil.RevSwerveModuleConstants;

/**
 * This file comes with command robot projects, and is intended to contain
 * configuration information.
 * I think it would be prudent if this file only contained CanIDs, because it
 * is useful to have all the ids for the whole robot in one place.
 * other configuration goes into subsystem specific configuration files,
 * to make sure this one isn't cluttered.
 */
public final class SwerveConstants {

    public static final double stickDeadband = 0.1;
    public static final double limelightOffset = 3;

    public static final class REV {

        public static final int pigeonID = 17;
    }

    public static final class Swerve {

        /* Module Specific Constants */
        /* Front Left Module */
        public static final class Mod0 {

            public static final int driveMotorID = 4;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 13;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(11.074219); //Rotation2d.fromDegrees(37.7);
            public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(
                driveMotorID,
                angleMotorID,
                canCoderID,
                angleOffset
            );
        }

        /* Front Right Module */
        public static final class Mod1 {

            public static final int driveMotorID = 6;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((79.277344) + 180.0);
            public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(
                driveMotorID,
                angleMotorID,
                canCoderID,
                angleOffset
            );
        }

        /* Back Left Module */
        public static final class Mod2 {

            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((73.740234));
            public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(
                driveMotorID,
                angleMotorID,
                canCoderID,
                angleOffset
            );
        }

        /* Back Right Module */
        public static final class Mod3 {

            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees((185.273438) + 180.0);
            public static final RevSwerveModuleConstants constants = new RevSwerveModuleConstants(
                driveMotorID,
                angleMotorID,
                canCoderID,
                angleOffset
            );
        }
    }
}
