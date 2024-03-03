// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.awt.geom.Point2D;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.08;
    public static final double kCubic = 0.95;
    public static final double kLinear = 0.05;
  }

  public static final class CurrentLimit {
    public static final int kDriveStator = 100;
    public static final int kDriveSupply = 50;
    public static final int kAzimuth = 20;
    public static final int kShooter = 55;
    public static final int kFeeder = 40;
    public static final int kIntaker = 30;
    public static final int kKicker = 20;
    public static final int kIndexer = 40;
    public static final int kElevator = 40;
    public static final int kClimber = 60;
    public static final int kManipulator = 30;
    public static final int kPitcher = 20;
  }

  public static final class ModuleConstants {
    public static final class Drive {
      public static final double kGearRatio = (36.0 / 14.0) * (18.0 / 24.0) * (45.0 / 15.0);
      public static final double kWheelDiameter = 0.09741;
      public static final double kToMeters = (1.0 / kGearRatio) * kWheelDiameter * Math.PI;
      public static final double kToRots = 1 / kToMeters;
      public static final double kKrakenMaxRPS = 100.0;
    }

    public static final class Aziumth {
      public static final double kGearRatio = (50.0 / 12.0) * (72.0 / 12.0);
      public static final double kPositionFactor = 1 / (kGearRatio) * 2 * Math.PI;
      public static final double kVelocityFactor = kPositionFactor / 60.0;
      public static final double kp = 1.0;
      public static final double rioKp = 0.8;
      public static final double rioKi = 0.0;
      public static final double rioKd = 0.0;

    }

  }

  public static final class VisionConstants {
    public static final double kPoseErrorAcceptance = 2.0; // How much error there can be between current stimated pose
                                                           // and vision pose in meters
  }

  public static final class DriveConstants {

    public static final double kWheelBaseWidth = 0.5842;
    public static final double kWheelBaseLength = 0.5334;
    public static final double kWheelBaseRadius = 0.5
        * Math.sqrt(Math.pow(kWheelBaseLength, 2) + Math.pow(kWheelBaseWidth, 2));

    public static final class FrontLeft {
      public static final int kModuleID = 4;
      public static final double kOffset = -4.217;
      public static final Translation2d kLocation = new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2);
    }

    public static final class FrontRight {
      public static final int kModuleID = 1;
      public static final double kOffset = -5.191;
      public static final Translation2d kLocation = new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2);
    }

    public static final class RearLeft {
      public static final int kModuleID = 3;
      public static final double kOffset = -3.136;
      public static final Translation2d kLocation = new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2);
    }

    public static final class RearRight {
      public static final int kModuleID = 2;
      public static final double kOffset = -2.264;
      public static final Translation2d kLocation = new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2);
    }

    public static final double kTransSlewRate = 12.0;
    public static final double kRotSlewRate = 16.0;

    public static final double kMaxSpeedMetersPerSecond = 5.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    public static final double kMaxAngularAccel = 1.5 * Math.PI;

    public static final class KeepAngle {
      public static final double kp = 0.50;
      public static final double ki = 0.0;
      public static final double kd = 0.0;
    }

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(FrontLeft.kLocation,
        FrontRight.kLocation, RearLeft.kLocation, RearRight.kLocation);

    public static final double kRotTransFactor = 0.045;

    public static final class Auto {

      public static final HolonomicPathFollowerConfig autoConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig,
                                                                                                    // this should
                                                                                                    // likely live in
                                                                                                    // your
          // Constants class
          new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
          new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
          5.0, // Max module speed, in m/s
          DriveConstants.kWheelBaseRadius, // Drive base radius in meters. Distance from robot center to furthest
                                           // module.
          new ReplanningConfig() // Default path replanning config. See the API for the options here
      );
    }
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }

  public static final class PitcherConstants {
    public static final float kMax = (float) 20.0;
    public static final float kMin = (float) 2.0;
    public static final double kP = 0.1;
  }

  public static final class ShooterConstants {
    public static final Point2D[] kDistTable = {
        new Point2D.Double(16.75,38.0),
        new Point2D.Double(12.15, 52.25),
        new Point2D.Double(8.95, 56.0),
        new Point2D.Double(6.11, 44.625+38.0),
        new Point2D.Double(3.76, 66.0+38.0),
        new Point2D.Double(2.17, 82.0+38.0),
        new Point2D.Double(0.23, 107.0+38.0),
        new Point2D.Double(-1.33, 128.5+38.0),
        new Point2D.Double(-2.49, 150.5+38.0),
        new Point2D.Double(-3.57, 174.0+38.0),
        new Point2D.Double(-3.99, 196.0+38.0),
        new Point2D.Double(-4.7, 221.5+38.0)
    };

    public static final Point2D[] kPitchTable = {
        new Point2D.Double(41.0,20.0),
        new Point2D.Double(53.68,18.50),
        new Point2D.Double(63.77,16.67),
        new Point2D.Double(85.77,14.97),
        new Point2D.Double(100.2,13.41),
        new Point2D.Double(120.0,12.40),
        new Point2D.Double(140.0,10.54),
        new Point2D.Double(160.5,9.05),
        new Point2D.Double(171.0,7.86),
        new Point2D.Double(179.0,7.27),
        new Point2D.Double(200.0,6.51)
    };

    public static final Point2D[] kVelocityTable = {
        new Point2D.Double(41.0,47.0),
        new Point2D.Double(53.68,47.0),
        new Point2D.Double(63.77,47.0),
        new Point2D.Double(85.77,47.0),
        new Point2D.Double(100.2,51.33),
        new Point2D.Double(120.0,53.25),
        new Point2D.Double(140.0,56.03),
        new Point2D.Double(160.5,60.09),
        new Point2D.Double(171.0,70.13),
        new Point2D.Double(179.0,80.54),
        new Point2D.Double(200.0,85.6)
    };

    public static final Point2D[] kTimeTable = {
        new Point2D.Double(1.0, 0.3),
        new Point2D.Double(3.0, 0.35),
        new Point2D.Double(5.0, 0.4)
    };

  }

}
