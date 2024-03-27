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
    public static final double kDeadband = 0.01;
    public static final double kCubic = 0.95;
    public static final double kLinear = 0.05;
  }

  public static final class CurrentLimit {
    public static final int kDriveStator = 120;
    public static final int kDriveSupply = 60;
    public static final int kAzimuth = 20;
    public static final int kShooterSupply = 40;
    public static final int kShooterStator = 80;
    public static final int kFeeder = 40;
    public static final int kIntakerSupply = 30;
    public static final int kIntakerStator = 60;
    public static final int kKicker = 20;
    public static final int kIndexer = 40;
    public static final int kElevator = 40;
    public static final int kClimber = 60;
    public static final int kManipulator = 30;
    public static final int kPitcher = 30;
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
    public static final double kPoseErrorAcceptance = 3.0; // How much error there can be between current stimated pose
                                                           // and vision pose in meters
  }

  public static final class GoalConstants{
    public static final Translation2d kRedGoal = new Translation2d(643.23/39.37, 218.42/39.37);
    public static final Translation2d kBlueGoal = new Translation2d(8.00/39.37, 218.42/39.37);
    public static final Translation2d kRedFeed = new Translation2d(594.23/39.37, 236.42/39.37);
    public static final Translation2d kBlueFeed = new Translation2d(57.0/39.37, 236.42/39.37);
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
      public static final double kOffset = -3.136+0.5236;
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

  public static final class ElevatorConstants {
    public static final double kRest = 2.7;
  }

  public static final class ShooterConstants {
    private static final double kpowerOffset = 0.0;
    private static final double kPitchOffset = 0.2;
    public static final Point2D[] kDistTable = {
        new Point2D.Double(0.20,38.0),//.3B
        new Point2D.Double(-3.36, 38.0+12.0),
        new Point2D.Double(-6.38, 38.0+24.0),//-6.35B
        new Point2D.Double(-9.00, 38.0+36.0),
        new Point2D.Double(-10.80, 38.0+48.0),//-10.65,-10.93B
        new Point2D.Double(-11.90, 38.0+60.0),
        new Point2D.Double(-13.60, 38.0+72.0),//-13.50,-13.7B
        new Point2D.Double(-14.61, 38.0+84.0),
        new Point2D.Double(-15.54,38.0+96.0),//-15.58, -15.50B
        new Point2D.Double(-16.05, 38.0+108),
        new Point2D.Double(-17.05,38.0+120.0),//-17.00,-17.1B
        new Point2D.Double(-17.50, 38.0+132.0),
        new Point2D.Double(-18.00,38.0+144.0),//-17.80,-18.17B
        new Point2D.Double(-18.45, 38.0+156.0),
        new Point2D.Double(-18.81, 38.0+168.0),
        new Point2D.Double(-19.27,38.0+180.0)
    };

    public static final Point2D[] kPitchTable = {
        new Point2D.Double(42.0, 20.0 + kPitchOffset),
        new Point2D.Double(42.0+12.0, 19.0+ kPitchOffset),
        new Point2D.Double(42.0+24.0, 18.0 + kPitchOffset),
        new Point2D.Double(42.0+36.0, 15.0 + kPitchOffset),
        new Point2D.Double(42.0+48.0, 14.0 + kPitchOffset),
        new Point2D.Double(42.0+60.0, 12.8 + kPitchOffset),
        new Point2D.Double(42.0+72.0, 12.0 + kPitchOffset),
        new Point2D.Double(42.0+84.0, 11.5 + kPitchOffset),
        new Point2D.Double(42.0+96.0, 10.4 + kPitchOffset),
        new Point2D.Double(42.0+108.0, 9.5 + kPitchOffset),
        new Point2D.Double(42.0+120.0, 8.70 + kPitchOffset),
        new Point2D.Double(42.0+132.0, 8.05 + kPitchOffset),
        new Point2D.Double(42.0+144.0, 7.3 + kPitchOffset),
        new Point2D.Double(42.0+156.0, 6.50 + kPitchOffset),
        new Point2D.Double(42.0+168.0, 6.49 + kPitchOffset),
        new Point2D.Double(42.0+180.0, 6.15 + kPitchOffset), 
        new Point2D.Double(236.0,5.9 + kPitchOffset),
        new Point2D.Double(260.0,5.7 + kPitchOffset),
        new Point2D.Double(283.0,5.4 + kPitchOffset)
    };

    public static final Point2D[] kAutoPitchTable = {
        new Point2D.Double(42.0, 20.0 + kPitchOffset),
        new Point2D.Double(42.0+12.0, 19.6+ kPitchOffset),
        new Point2D.Double(42.0+24.0, 18.6 + kPitchOffset),
        new Point2D.Double(42.0+36.0, 15.0 + kPitchOffset),
        new Point2D.Double(42.0+48.0, 14.0 + kPitchOffset),
        new Point2D.Double(42.0+60.0, 12.8 + kPitchOffset),
        new Point2D.Double(42.0+72.0, 12.0 + kPitchOffset),
        new Point2D.Double(42.0+84.0, 11.5 + kPitchOffset),
        new Point2D.Double(42.0+96.0, 10.4 + kPitchOffset),
        new Point2D.Double(42.0+108.0, 9.5 + kPitchOffset),
        new Point2D.Double(42.0+120.0, 8.70 + kPitchOffset),
        new Point2D.Double(42.0+132.0, 8.05 + kPitchOffset),
        new Point2D.Double(42.0+144.0, 7.3 + kPitchOffset),
        new Point2D.Double(42.0+156.0, 6.50 + kPitchOffset),
        new Point2D.Double(42.0+168.0, 6.49 + kPitchOffset),
        new Point2D.Double(42.0+180.0, 6.15 + kPitchOffset), 
        new Point2D.Double(236.0,5.9 + kPitchOffset),
        new Point2D.Double(260.0,5.7 + kPitchOffset),
        new Point2D.Double(283.0,5.4 + kPitchOffset)
    };

    public static final Point2D[] kVelocityTable = {
        new Point2D.Double(42.0, 52.06 + kpowerOffset),
        new Point2D.Double(42.0+12.0, 52.06+ kpowerOffset),
        new Point2D.Double(42.0+24.0, 52.06+ kpowerOffset),
        new Point2D.Double(42.0+36.0, 52.06+ kpowerOffset),
        new Point2D.Double(42.0+48.0, 52.06+ kpowerOffset),
        new Point2D.Double(42.0+60.0, 52.06+ kpowerOffset),
        new Point2D.Double(42.0+72.0, 52.06+ kpowerOffset),
        new Point2D.Double(42.0+84.0, 52.48+ kpowerOffset),
        new Point2D.Double(42.0+96.0, 56.0+ kpowerOffset),
        new Point2D.Double(42.0+108.0, 60.0+ kpowerOffset),
        new Point2D.Double(42.0+120.0, 61.35+ kpowerOffset),
        new Point2D.Double(42.0+132.0, 65.23+ kpowerOffset),
        new Point2D.Double(42.0+144.0, 70.0+ kpowerOffset),
        new Point2D.Double(42.0+156.0, 75.00+ kpowerOffset),
        new Point2D.Double(42.0+168.0, 75.00+ kpowerOffset),
        new Point2D.Double(42.0+180.0, 75.00+ kpowerOffset)
    };

        public static final Point2D[] kAutoVelocityTable = {
        new Point2D.Double(42.0, 45.00 + kpowerOffset),
        new Point2D.Double(42.0+12.0, 45.00+ kpowerOffset),
        new Point2D.Double(42.0+24.0, 45.00+ kpowerOffset),
        new Point2D.Double(42.0+36.0, 45.00+ kpowerOffset),
        new Point2D.Double(42.0+48.0, 45.00+ kpowerOffset),
        new Point2D.Double(42.0+60.0, 52.06+ kpowerOffset),
        new Point2D.Double(42.0+72.0, 52.06+ kpowerOffset),
        new Point2D.Double(42.0+84.0, 52.48+ kpowerOffset),
        new Point2D.Double(42.0+96.0, 56.0+ kpowerOffset),
        new Point2D.Double(42.0+108.0, 60.0+ kpowerOffset),
        new Point2D.Double(42.0+120.0, 61.35+ kpowerOffset),
        new Point2D.Double(42.0+132.0, 65.23+ kpowerOffset),
        new Point2D.Double(42.0+144.0, 70.0+ kpowerOffset),
        new Point2D.Double(42.0+156.0, 75.00+ kpowerOffset),
        new Point2D.Double(42.0+168.0, 75.00+ kpowerOffset),
        new Point2D.Double(42.0+180.0, 75.00+ kpowerOffset)
    };

    public static final Point2D[] kTimeTable = {
        new Point2D.Double(1.0, 0.28),
        new Point2D.Double(3.0, 0.35),
        new Point2D.Double(5.0, 0.40)
    };

        public static final Point2D[] kFeedPitch = {
        new Point2D.Double(320.0, 20.0),
        new Point2D.Double(380.0, 13.0)
    };

    public static final Point2D[] kFeedVelocity = {
        new Point2D.Double(320.0, 40.0),
        new Point2D.Double(380.0, 50.0)
    };

    public static final Point2D[] kFeedTime = {
        new Point2D.Double(6.0, 0.9),
        new Point2D.Double(8.0, 1.0),
        new Point2D.Double(10.0, 1.1)
    };

  }

}
