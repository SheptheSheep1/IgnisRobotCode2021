// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.PIDShooter;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static final Drivetrain m_drivetrain = new Drivetrain();
  public static final Shooter m_shooter = new Shooter();
  public static final XboxController m_driverController = new XboxController(0);
  public static final Intake m_intake = new Intake();
  public static final Hopper m_hopper = new Hopper();
  private static RobotContainer m_robotContainer = new RobotContainer();
  public static final Limelight m_limelight = new Limelight();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    
    configureButtonBindings();
   
    m_drivetrain.setDefaultCommand(new RunCommand(() ->
    m_drivetrain.arcadeDrive(
      m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrain));
  
      /*
    m_shooter.setDefaultCommand(new RunCommand(() ->
                  m_shooter.manualSpinMotor(
                    m_driverController.getRightTrigger()), m_shooter));
                  */ 
                  /*
      m_intake.setDefaultCommand(new RunCommand(() -> 
                  m_intake.setIntake(
                    m_driverController.getLeftY()), m_intake));
              */ 
             /*
    m_hopper.setDefaultCommand(new RunCommand(() ->
                        m_hopper.setHopper(m_driverController.getLeftY()), m_hopper));
*/
    // Configure the button bindings
  
  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  
  private void configureButtonBindings() {
    /*
    new JoystickButton(m_driverController, Button.kA.value)
        .whenHeld(
            new PIDCommand(new PIDController(.175, 0, .1),
            // Close loop on heading
            Limelight::getTx,
            // Set reference to target
            0,
            // Pipe output to turn robot
            output -> m_drivetrain.arcadeDrive(0.0, output), m_drivetrain));
            */
            /*
        new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(new RunCommand(() -> m_shooter.manualSpinMotor(0), m_shooter));
        */
/*
        new JoystickButton(m_driverController, Button.kB.value)
        .whenPressed(new PIDShooter(m_shooter).withTimeout(3));
*/
        new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new DriveToDistance(m_drivetrain).withTimeout(3));

        new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new TurnToTarget(0, m_drivetrain).withTimeout(3));
        
        new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whenPressed(new RunCommand(() -> m_intake.setIntake(-.2), m_intake))
        .whenReleased(new RunCommand(() -> m_intake.setIntake(0), m_intake));
        
        new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(new RunCommand(() -> m_drivetrain.baseDriveTo(100)));

    //Intake System
    /*
    new JoystickButton(m_driverController, Button.kB.value)
                        .whileHeld(() -> {
                                m_intake.setIntake(IntakeConstants.kIntakeSpeed);
                                m_hopper.setHopper(HopperConstants.kHopperSpeed);
                        }, m_intake, m_hopper)
                        .whenReleased(() -> {
                                m_intake.setIntake(0);
                                m_hopper.setHopper(0);
                        }, m_intake, m_hopper);
*/
  //Hopper System
  //May have to input solenoid boolean values
  /*
  new JoystickButton(m_driverController, Button.kA.value)
                        .whenPressed(() -> m_hopper.toggleSolenoid(), m_hopper)
                        .whileHeld(() -> m_hopper.setHopper(HopperConstants.kHopperSpeed), m_hopper)
                        .whenReleased(() -> {
                                m_hopper.setHopper(0);
                                m_hopper.toggleSolenoid();
                        }, m_hopper);
                new JoystickButton(m_driverController, Button.kB.value)
                        .whileHeld(() -> m_hopper.toggleSolenoid(), m_hopper);
                        */

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getDrive() {
    return m_drive;
  }
  
  public Command getShoot() {
    return m_shoot;
  }
  */
  public static XboxController getXboxController() {
    return m_driverController;
  }
  
  public static RobotContainer getInstance() {
    return m_robotContainer;
  }

}