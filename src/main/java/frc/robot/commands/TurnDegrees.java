// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnDegrees extends PIDCommand {
  /** Creates a new TurnToAngle. */
  public TurnDegrees(double targetAngleDegrees, Drivetrain drive) {
    super(
        // The controller that the command will use
        new PIDController(.05, 0, 0),
        // This should return the measurement
        drive::getAngle,
        // This should return the setpoint (can also be a constant)
        targetAngleDegrees,
        // This uses the output may have to be negative
        output -> { drive.arcadeDrive(0, MathUtil.clamp(output, -1, 1));
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    getController().enableContinuousInput(-180, 180);
    getController().setTolerance(5);
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
