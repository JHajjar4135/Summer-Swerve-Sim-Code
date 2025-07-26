// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.SwerveSlewRateLimiter;

public class SwerveDriveSub extends SubsystemBase {
  /** Creates a new SwerveDriveSub. */
  public SwerveDriveSub() {}
  // private final SwerveModule m_frontLeftModule = new SwerveModule(
  //   k_frontLeftDrive,
  //   k_frontLeftSteer, 
  //   k_frontLeftEncoder, 
  //   k_frontLeftMagnetOffset
  //   );

  //   private final SwerveModule m_frontRightModule = new SwerveModule(
  //   k_frontRightDrive,
  //   k_frontRightSteer,
  //   k_frontRightEncoder,
  //   k_frontRightMagnetOffset
  // );

  // private final SwerveModule m_backLeftModule = new SwerveModule(
  //   k_backLeftDrive,
  //   k_backLeftSteer,
  //   k_backLeftEncoder,
  //   k_backLeftMagnetOffset
  // );

  // private final SwerveModule m_backRightModule = new SwerveModule(
  //   k_backRightDrive,
  //   k_backRightSteer,
  //   k_backRightEncoder,
  //   k_backRightMagnetOffset
  // );

  // private final SwerveSlewRateLimiter m_driveSlewRateLimiter = new SwerveSlewRateLimiter(
  //   k_linearSlewRate, 
  //   k_negLinearSlewRate, 
  //   k_rotationalSlewRate, 
  //   k_negRotationalSlewRate
  //   );

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
