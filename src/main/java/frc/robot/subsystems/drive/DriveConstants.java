// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/*
! Things that need to be configured in addition to AdvantageKit Swerve Template configs

* maxAngularSpeedFactor
   - Units: rad/sec
   - Divide max rotation speed when driving by max rotation speed while stationary

* pigeonYawPositionFactor
   - Units: Unit: rad
   - Divide measured yaw from Pigeon after 30 turns by expected reading after 30 turns
*/

public class DriveConstants {
  public static final double maxSpeedMetersPerSec = 4.8;
  // * Max rotation speed (Rad/Sec) while moving / Max rotation speed while stationary
  public static final double maxAngularSpeedFactor = (5.48598 / 9.52601);
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(22);
  public static final double wheelBase = Units.inchesToMeters(22);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final double wheelRadiusMeters =
      Units.inchesToMeters(1.57); // TODO: between 1.53 and 1.57
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final int frontLeftTurnEncoderId = 0;
  public static final int frontRightTurnEncoderId = 1;
  public static final int backLeftTurnEncoderId = 2;
  public static final int backRightTurnEncoderId = 3;

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(5.547);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(1.203);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(6.262);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(2.293);

  // Device CAN IDs
  public static final int pigeonCanId = 20;

  public static final int frontLeftDriveCanId = 1;
  public static final int backLeftDriveCanId = 15;
  public static final int frontRightDriveCanId = 30;
  public static final int backRightDriveCanId = 14;

  public static final int frontLeftTurnCanId = 3;
  public static final int backLeftTurnCanId = 2;
  public static final int frontRightTurnCanId = 12;
  public static final int backRightTurnCanId = 11;

  // Drive motor configuration
  public static final boolean driveInverted = false;
  public static final int driveMotorCurrentLimit = 50;
  public static final double driveMotorReduction =
      (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // Mk4i L2
  // Gearing
  // and 22 spur teeth
  public static final DCMotor driveGearbox = DCMotor.getNeoVortex(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorReduction; // Rotor Rotations ->
  // Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorReduction; // Rotor RPM
  // ->
  // Wheel Rad/Sec

  // Drive PID configuration
  public static final double driveKp = 0.0;
  public static final double driveKd = 0.0;
  public static final double driveKs = 0.13243;
  public static final double driveKv = 0.11346;
  public static final double driveSimP = 0.05;
  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.0;
  public static final double driveSimKv = 0.0789;

  // Turn motor configuration
  public static final boolean turnInverted = true;
  public static final int turnMotorCurrentLimit = 20;
  public static final double turnMotorReduction = ((150.0 / 7.0) / (2.0 * Math.PI));
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
  public static final boolean turnEncoderInverted = true;
  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
  public static final double turnKp = 0.5;
  public static final double turnKd = 0.0;
  public static final double turnSimP = 8.0;
  public static final double turnSimD = 0.0;
  public static final double turnPIDMinInput = 0; // Radians
  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  // Pigeon configuration
  // * Measured yaw (rad) after # of turns / # of turns
  public static final double pigeonYawPositionFactor =
      ((21 * 2 * Math.PI + -1.2856) / (21 * 2 * Math.PI));

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              wheelRadiusMeters,
              maxSpeedMetersPerSec,
              wheelCOF,
              driveGearbox.withReduction(driveMotorReduction),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  // Auto config
  public static final double autoLinearKp = 10.0;
  public static final double autoAngularKp = 7.5;
}
