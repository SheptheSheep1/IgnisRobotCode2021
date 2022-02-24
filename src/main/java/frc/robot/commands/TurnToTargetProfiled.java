// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTargetProfiled extends ProfiledPIDCommand {
  /** Creates a new TurnToTargetProfiled. */
  public TurnToTargetProfiled(double targetAngleDegrees, Drivetrain m_drive) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            .05,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(.5, .5)),
        //This should return the measurement
        Limelight::getTx,
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(targetAngleDegrees, 0),
        //This uses the output
        (output, setpoint) -> m_drive.arcadeDrive(0, output), 
        m_drive);
          // Use the output (and setpoint, if desired) here
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(1, 0);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
