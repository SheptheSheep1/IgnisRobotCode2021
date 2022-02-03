// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnToTarget extends PIDCommand {
  //Limelight m_limelight = new Limelight();
  //XboxController m_driverController = new XboxController(0);
  /** Creates a new TurnToTarget. */
  public TurnToTarget(double targetAngleDegrees, Drivetrain drive) {
    super(
        // The controller that the command will use
        new PIDController(.05, .03, .25),
        // Close loop on heading
        Limelight::getTx,
        // Set reference to target
        targetAngleDegrees,
        // Pipe output to turn robot
        output -> drive.arcadeDrive(0.0, -output),
        // Require the drive
        drive);

    getController().enableContinuousInput(-180, 180);
    
    //getController().setTolerance(1);
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
   getController().setTolerance(1, 0);
   //System.out.println(getController().calculate(Limelight.getTx(), targetAngleDegrees));
   //System.out.println(getController().toString());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
