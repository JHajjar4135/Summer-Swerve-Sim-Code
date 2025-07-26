// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import static frc.robot.Constants.*;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

/** Add your docs here. */
public class SwerveModule {
    private final SparkMax m_driveMotor;
    private final RelativeEncoder m_driveEncoder;
    private final SparkMax m_steerMotor;
    private final CANcoder m_steerEncoder;
    private final SparkClosedLoopController m_drivePIDController;
    private final PIDController m_steerPIDController;
    private final MagnetSensorConfigs m_canCoderConfig;

    public SwerveModule (int driveMotorID, int steerMotorID, int steerEncoderID, double magOffset){
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

        m_drivePIDController = m_driveMotor.getClosedLoopController();

        m_driverConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(k_swerveDrivePID[0])
            .i(k_swerveDrivePID[1])
            .d(k_swerveDrivePID[2])
            .velocityFF(k_swerveDrivePID[3]);

        m_driveMotor.configure(m_driverConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_steerMotor = new SparkMax(steerMotorID, MotorType.kBrushless);
        SparkMaxConfig m_steerConfig = new SparkMaxConfig();
        m_steerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        m_steerMotor.configure(m_steerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_steerPIDController = new PIDController(
            k_swerveSteerPID[0],
            k_swerveSteerPID[1],
            k_swerveSteerPID[2]);

        m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        m_steerEncoder = new CANcoder(steerEncoderID);

        m_canCoderConfig = new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withMagnetOffset(magOffset)
            .withAbsoluteSensorDiscontinuityPoint(k_AbsoluteSensorDiscontinuityPoint);
        m_steerEncoder.getConfigurator().apply(m_canCoderConfig);

    }

    private Rotation2d getWheelAngle(){
        return new Rotation2d(2*Math.PI*m_steerEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(m_driveEncoder.getPosition(), getWheelAngle());
    }

    public void setDesiredStates(SwerveModuleState desiredState){
        desiredState.optimize(getWheelAngle());

        double desiredWheelAngle = desiredState.angle.getRadians();
        double currentWheelAngleRadians = getWheelAngle().getRadians();

        double steerMotorPower = m_steerPIDController.calculate(currentWheelAngleRadians, desiredWheelAngle);
        steerMotorPower = Math.max(-1.0, Math.min(1.0, steerMotorPower));
        m_steerMotor.set(steerMotorPower);

        double WheelSpeeds = desiredState.speedMetersPerSecond;
        WheelSpeeds *= desiredState.angle.minus(getWheelAngle()).getCos();
        
        m_drivePIDController.setReference(WheelSpeeds, ControlType.kVelocity);
    }
}
