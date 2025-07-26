// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
    // m_robotDrive.setDefaultCommand(
    //   new RunCommand(() -> {
    //     m_robotDrive.drive(
    //       m_driveController.getLeftY(),
    //       m_driveController.getLeftX(),
    //       m_driveController.getRightX(),
    //     true
    //     );
    //   },
    //   m_robotDrive
    //   )
    // );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
