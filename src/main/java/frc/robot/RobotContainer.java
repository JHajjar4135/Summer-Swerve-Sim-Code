// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.k_controllerPort;
import static frc.robot.Constants.k_deadband;
import static frc.robot.Constants.k_debounce;
import static frc.robot.Constants.k_targetPosition;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.SwerveDriveSub;
import frc.robot.Commands.PointToTargedCmd;
import frc.robot.Utils.RealPS4Controller;

public class RobotContainer {
    private final SwerveDriveSub m_robotDrive = new SwerveDriveSub();
    private final RealPS4Controller m_driveController = new RealPS4Controller(
      k_controllerPort,
      k_deadband,
      k_debounce
      );

private final Trigger square = m_driveController.square();

  public RobotContainer() {
    configureBindings();
    m_robotDrive.setDefaultCommand(
      new RunCommand(() -> {
        m_robotDrive.drive(
          m_driveController.getLeftY(),
          m_driveController.getLeftX(),
          -m_driveController.getRightX(),
        true
        );
      },
      m_robotDrive
      )
    );
  }

  private void configureBindings() {
    PointToTargedCmd pointToTarget = new PointToTargedCmd(m_robotDrive,m_driveController,k_targetPosition);
    square.whileTrue(pointToTarget);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
