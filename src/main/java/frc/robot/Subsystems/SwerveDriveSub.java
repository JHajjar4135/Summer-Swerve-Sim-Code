// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import static frc.robot.Constants.*;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.SwerveModule;
import frc.robot.Utils.SwerveSlewRateLimiter;

public class SwerveDriveSub extends SubsystemBase {
  /** Creates a new SwerveDriveSub. */
  public SwerveDriveSub() {}
  private final SwerveModule m_frontLeftModule = new SwerveModule(
    k_frontLeftDrive,
    k_frontLeftSteer, 
    k_frontLeftEncoder, 
    k_frontLeftMagnetOffset
    );

    private final SwerveModule m_frontRightModule = new SwerveModule(
    k_frontRightDrive,
    k_frontRightSteer,
    k_frontRightEncoder,
    k_frontRightMagnetOffset
  );

  private final SwerveModule m_backLeftModule = new SwerveModule(
    k_backLeftDrive,
    k_backLeftSteer,
    k_backLeftEncoder,
    k_backLeftMagnetOffset
  );

  private final SwerveModule m_backRightModule = new SwerveModule(
    k_backRightDrive,
    k_backRightSteer,
    k_backRightEncoder,
    k_backRightMagnetOffset
  );

  private final SwerveSlewRateLimiter m_driveSlewRateLimiter = new SwerveSlewRateLimiter(
    k_linearSlewRate, 
    k_negLinearSlewRate, 
    k_rotationalSlewRate, 
    k_negRotationalSlewRate
    );

  private AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);
  private boolean m_isGyroZeroed = false;
  private final SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[4];
  private final SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(
    k_kinematics,
    getHeading(), 
    getSwerveModulePositions()
  );
private ChassisSpeeds m_chassisSpeeds;

  public SwerveModulePosition[] getSwerveModulePositions() {
    m_swerveModulePositions[0] = m_frontLeftModule.getPosition();
    m_swerveModulePositions[1] = m_frontRightModule.getPosition();
    m_swerveModulePositions[2] = m_backLeftModule.getPosition();
    m_swerveModulePositions[3] = m_backRightModule.getPosition();
    return m_swerveModulePositions;
  }


  public Rotation2d getHeading(){
    return Rotation2d.fromDegrees(-m_gyro.getYaw());
  }

  public void resetOdometry(Pose2d pose) {
    m_Odometry.resetPosition(
      getHeading(),
      getSwerveModulePositions(),
      pose);
  }
  public Pose2d getPose2d() {
    return m_Odometry.getPoseMeters();
  }

  public void drive(double xSpeed, double ySpeed, double omega, boolean fieldRelative){
    double[] commandedSpeeds = m_driveSlewRateLimiter.calculateSlewRate(xSpeed, ySpeed, omega);

    xSpeed = commandedSpeeds[0];
    ySpeed = commandedSpeeds[1];
    omega = commandedSpeeds[2];

    double xSpeedCommanded = xSpeed * k_maxMetersPerSec;
    double ySpeedCommanded = ySpeed * k_maxMetersPerSec;
    double omegaCommanded = omega * k_maxMetersPerSec;

    if(fieldRelative){
      m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeedCommanded,
        ySpeedCommanded,
        omegaCommanded,
          getHeading()
      );
    } else {
      m_chassisSpeeds = new ChassisSpeeds(
        xSpeedCommanded,
        ySpeedCommanded,
        omegaCommanded
      );
    }
    ChassisSpeeds discritizedChassisSpeeds = ChassisSpeeds.discretize(m_chassisSpeeds, k_drivePeriod);
    SwerveModuleState[] swerveModuleStates = k_kinematics.toSwerveModuleStates(discritizedChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, k_maxMetersPerSec);

    m_frontLeftModule.setDesiredStates(swerveModuleStates[0]);
    m_frontRightModule.setDesiredStates(swerveModuleStates[1]);
    m_backLeftModule.setDesiredStates(swerveModuleStates[2]);
    m_backRightModule.setDesiredStates(swerveModuleStates[3]);
    
  }

  @Override
  public void periodic() {
    if(!m_gyro.isCalibrating() && !m_isGyroZeroed){
      m_gyro.enableBoardlevelYawReset(true);
      m_gyro.reset();
      m_isGyroZeroed = true;
    }

    m_Odometry.update(
    getHeading(),
    getSwerveModulePositions()
    );
  }
}
