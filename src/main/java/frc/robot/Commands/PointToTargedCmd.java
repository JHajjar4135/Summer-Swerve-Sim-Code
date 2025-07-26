// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static frc.robot.Constants.k_pointToTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDriveSub;
import frc.robot.Utils.RealPS4Controller;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointToTargedCmd extends Command {
  private final SwerveDriveSub m_robotDrive;
  private final RealPS4Controller m_Ps4Controller;
  private final Translation2d m_targetPosition;
  private final PIDController m_targetController;

  /** Creates a new PointToTargedCmd. */
  public PointToTargedCmd(SwerveDriveSub robotDrive, RealPS4Controller ps4Controller, Translation2d targetPosition) {
    this.m_robotDrive = robotDrive;
    this.m_Ps4Controller = ps4Controller;
    this.m_targetPosition = targetPosition;
    m_targetController = new PIDController(k_pointToTarget[0],
    k_pointToTarget[1],
    k_pointToTarget[2]
    );
    m_targetController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_robotDrive.getPose2d();
    Translation2d currentTranslation = currentPose.getTranslation();
    double targetAngle = Math.atan2(
      m_targetPosition.getY() - currentTranslation.getY(),
      m_targetPosition.getX() - currentTranslation.getX()
    );

    double currentAngle = currentPose.getRotation().getRadians();

    double targetCorrection = m_targetController.calculate(currentAngle, targetAngle);
    targetCorrection = Math.max(-1.0, Math.min(1.0, targetCorrection));
    
    m_robotDrive.drive(m_Ps4Controller.getLeftY(), m_Ps4Controller.getLeftX(), targetCorrection,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
