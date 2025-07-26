// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import static frc.robot.Constants.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class SwerveModule {
    private final SparkMax m_driveMotor;
    private final RelativeEncoder m_driveEncoder;
    private final SparkMax m_steerMotor;
    private final CANcoder m_steerEncoder;

    public SwerveModule (int driveMotorID, int steerMotorID, int steerEncoderID){
        m_driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        SparkMaxConfig m_driverConfig = new SparkMaxConfig();
        m_driverConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        m_driverConfig.encoder
            .positionConversionFactor(k_drivingEncoderPositionFactor)
            .velocityConversionFactor(k_drivingEncoderVelocityFactor);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setPosition(0);

        m_driveMotor.configure(m_driverConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        SparkMaxConfig m_steerConfig = new SparkMaxConfig();
        m_steerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        m_steerMotor.configure(m_steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_steerEncoder = new CANcoder(steerEncoderID);

    }
}
