// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;
import edu.wpi.first.math.filter.SlewRateLimiter;

public class SwerveSlewRateLimiter {

    private SlewRateLimiter m_accelerationLimit;
    private SlewRateLimiter m_rotationAccelerationLimit;
    private double m_inputDirection = 0;

    public SwerveSlewRateLimiter(
        double linearSlewLimit, 
        double negLinearSlewLimit, 
        double rotationalSlewLimit, 
        double negRotationalSlewLimit ){

        m_accelerationLimit = new SlewRateLimiter(linearSlewLimit, negLinearSlewLimit,0);
        m_rotationAccelerationLimit = new SlewRateLimiter(rotationalSlewLimit, negRotationalSlewLimit,0);
    }

    public double[] calculateSlewRate(double xSpeed, double ySpeed, double omega){
        double inputTranslationMag = Math.hypot(xSpeed, ySpeed);
        if(inputTranslationMag>.01){
            m_inputDirection = Math.atan2(-ySpeed, -xSpeed);
        }

        double allowedTranslationMag = m_accelerationLimit.calculate(inputTranslationMag);

        double xSpeedCommanded = allowedTranslationMag * Math.cos(m_inputDirection);
        double ySpeedCommanded = allowedTranslationMag * Math.sin(m_inputDirection);
        double omegaCommanded = m_rotationAccelerationLimit.calculate(omega);

        return new double[]{xSpeedCommanded, ySpeedCommanded, omegaCommanded};
    }
}