// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.08;
    public static final double kCubic = 0.95;
    public static final double kLinear = 0.05;
  }

  public static final class CurrentLimit {
    public static final int kDriveStator = 120;
    public static final int kDriveSupply = 60;
    public static final int kAzimuth = 20;
    public static final int kShooter = 60;
    public static final int kFeeder = 40;
    public static final int kIntaker = 30;
    public static final int kKicker = 30;
    public static final int kIndexer = 40;
    public static final int kElevator = 40;
    public static final int kClimber = 60; 
    public static final int kManipulator = 40; 
    public static final int kPitcher = 30;
  }

  public static final class ModuleConstants {
    public static final class Drive {
      public static final double kGearRatio = (36.0 / 14.0) * (18.0 / 24.0) * (45.0 / 15.0);
      public static final double kWheelDiameter = 0.095;
      public static final double kToMeters = (1.0 / kGearRatio) * kWheelDiameter * Math.PI;
      public static final double kToRots = 1/kToMeters;
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

  public static final class DriveConstants {

    public static final double kWheelBaseWidth = 0.5842;
    public static final double kWheelBaseLength = 0.5334;
    public static final double kWheelBaseRadius = 0.5*Math.sqrt(Math.pow(kWheelBaseLength, 2) + Math.pow(kWheelBaseWidth, 2));

    public static final class FrontLeft {
      public static final int kModuleID = 4;
      public static final double kOffset = -1.086+Math.PI-6.271;
      public static final Translation2d kLocation = new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2);
    }

    public static final class FrontRight {
      public static final int kModuleID = 1;
      public static final double kOffset = -2.074+Math.PI-6.241;
      public static final Translation2d kLocation = new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2);
    }

    public static final class RearLeft {
      public static final int kModuleID = 3;
      public static final double kOffset = -0.006+2*Math.PI-9.341;
      public static final Translation2d kLocation = new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2);
    }

    public static final class RearRight {
      public static final int kModuleID = 2;
      public static final double kOffset = -5.408+Math.PI+0.037;
      public static final Translation2d kLocation = new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2);
    }

    public static final double kTransSlewRate = 14.0;
    public static final double kRotSlewRate = 16.0;

    public static final double kMaxSpeedMetersPerSecond = 5.1;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    public static final double kMaxAngularAccel = 1.5 * Math.PI;

    public static final class KeepAngle {
      public static final double kp = 0.30;
      public static final double ki = 0.0;
      public static final double kd = 0.0;
    }

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(FrontLeft.kLocation,
        FrontRight.kLocation, RearLeft.kLocation, RearRight.kLocation);

    public static final double kRotTransFactor = 0.045;

    public static final class Auto {
      public static final double kTranslationP = 1.0; // Placeholder, probaly should be changed
      public static final double kRotationP = 1.0; // Placeholder, probaly should be changed
    }
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }

  public static final class PitcherConstants{
    public static final float kMax = (float)20.0;
    public static final float kMin = (float)2.0;
    public static final double kP = 0.1;
  }


}
