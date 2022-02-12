// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.PathWeaver;
import frc.robot.commands.Auto;
import frc.robot.commands.DriveToDistance;
import frc.robot.commands.TurnToTarget;
import frc.robot.commands.TurnToTargetProfiled;
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
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command m_auto = new Auto(m_drivetrain, m_shooter);
  private final Command m_path = getAutonomousCommand("DriveToTarget");
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    configureButtonBindings();
   /*
    m_chooser.addOption("Autonomous", m_auto);
    m_chooser.addOption("PathWeaver", m_path);
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);
*/
    m_drivetrain.setDefaultCommand(new RunCommand(() ->
    m_drivetrain.arcadeDrive(
      m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrain));
  
  /*
    m_drivetrain.setDefaultCommand(new RunCommand(() -> 
    m_drivetrain.tankDrive(m_driverController.getLeftY(), m_driverController.getRightY()), m_drivetrain));
*/
/*
    m_drivetrain.setDefaultCommand(new RunCommand(() -> 
    m_drivetrain.curvatureDrive(m_driverController.getLeftY(), m_driverController.getRightX(), m_driverController.getAButton()), m_drivetrain));
    */
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
*//*
        new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(new DriveToDistance(100, m_drivetrain).withTimeout(3));
*/
        new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new TurnToTarget(0, m_drivetrain).withTimeout(3));
      /*
        new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new TurnToTargetProfiled(0, m_drivetrain));
        */
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
  public Command getAutonomousCommand(String path) {
    Trajectory exampleTrajectory = PathWeaver.getTrajectory(path); 
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            5);

    // Create config for trajectory
    
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);


    // An example trajectory to follow.  All units in meters.
    

    /*
       Trajectory exampleTrajectory =  TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
*/
  

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_drivetrain::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drivetrain.tankDriveVolts(0, 0));

  }


  public static XboxController getXboxController() {
    return m_driverController;
  }
  
  public static RobotContainer getInstance() {
    return m_robotContainer;
  }
  /*
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
  */
}