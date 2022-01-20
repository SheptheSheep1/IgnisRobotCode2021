// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  public final Drivetrain m_drivetrain;
  public final XboxController m_driverController;
 // private static final XboxController m_controller;
  public Drive(Drivetrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = subsystem;
        addRequirements(this.m_drivetrain);
       this.m_driverController = new XboxController(0);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.arcadeDrive(
                  m_driverController.getRightY(), m_driverController.getLeftX());
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
