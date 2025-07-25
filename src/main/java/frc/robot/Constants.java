// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;

/** Add your docs here. */
public class Constants {
    // motor ports
    public static final int k_frontLeftDrive = 19;
    public static final int k_frontLeftSteer = 18;
    public static final int k_frontRightDrive = 2;
    public static final int k_frontRightSteer = 1;
    public static final int k_backLeftDrive = 17;
    public static final int k_backLeftSteer = 16;
    public static final int k_backRightDrive = 4;
    public static final int k_backRightSteer = 3;

    // encoder ports
    public static final int k_frontLeftEncoder = 22;
    public static final int k_frontRightEncoder = 23;
    public static final int k_backLeftEncoder = 24;
    public static final int k_backRightEncoder = 21;

    // encoder magnet offset
    public static final double k_frontLeftMagnetOffset = -0.46044921875;
    public static final double k_frontRightMagnetOffset = -0.420654296875;
    public static final double k_backLeftMagnetOffset = -0.198974609375;
    public static final double k_backRightMagnetOffset = 0.45031250000;

    // swerv drive geometrical and mechanical constants
    public static final double k_wheelDiam = Units.inchesToMeters(4);
    public static final double k_swerveDriveGearRatio = 6.12;
    public static final double k_swerveSteerGearRatio = 150 / 7;
    public static final double k_drivingEncoderPositionFactor = (k_wheelDiam * Math.PI) / k_swerveDriveGearRatio;
    public static final double k_drivingEncoderVelocityFactor = k_drivingEncoderPositionFactor / 60;
    public static final double k_trackWidth = Units.inchesToMeters(23);
    public static final double k_wheelBase = Units.inchesToMeters(23);
    public static final SwerveDriveKinematics k_kinematics = new SwerveDriveKinematics(
            new Translation2d(k_wheelBase / 2, k_trackWidth / 2),
            new Translation2d(k_wheelBase / 2, -k_trackWidth / 2),
            new Translation2d(-k_wheelBase / 2, k_trackWidth / 2),
            new Translation2d(-k_wheelBase / 2, -k_trackWidth / 2));
    public static final double k_maxMetersPerSec = 4;
    public static final double k_maxOmegaRadiansPerSec = k_maxMetersPerSec
            / (.5 * Math.hypot(k_trackWidth, k_wheelBase));
    public static double k_AbsoluteSensorDiscontinuityPoint = .5;

    // PId constants
    public static final double[] k_swerveSteerPID = { .33, 0, 0 };
    public static final double[] k_swerveDrivePID = { 0.13, 0, 0, .194 };

    // Slew rate constants
    public static final double k_negLinearSlewRate = -1;
    public static final double k_linearSlewRate = 1;
    public static final double k_negRotationalSlewRate = -3;
    public static final double k_rotationalSlewRate = 3;

    // robot time period
    public static final double k_drivePeriod = TimedRobot.kDefaultPeriod;

    // driver controller constants
    public static final int k_controllerPort = 0;
    public static final double k_debounce = .15;
    public static final double k_deadband = .15;

    // point to target Constants
    public static final double[] k_pointToTarget = { 1.2, 0, 0 };
    public static final Translation2d k_targetPosition = new Translation2d(3, -3);

    public static final double[] k_goToTarget = {1.1, 0, 0};

    //algae grabber constants
public static final double k_algaeGrabberConversion = 20;       

    //Linear Slide Conversion Constants 
    public static final double k_linearSlidePositionConversionFactor = 2.175;

}
