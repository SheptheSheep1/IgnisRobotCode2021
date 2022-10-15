// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.PathWeaver;
import frc.robot.commands.Auto;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.SwitchDriveMode;
import frc.robot.commands.TurnDegrees;
import frc.robot.commands.TurnToTarget;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private enum CommandSelector {
    ONE, TWO, THREE
  }
  private CommandSelector select() {
    return CommandSelector.ONE;
  }
  public static final Drivetrain m_drivetrain = new Drivetrain();
  private static final Shooter m_shooter = new Shooter();
  public static final XboxController m_driverController = new XboxController(0);
  private static final Intake m_intake = new Intake();
  private static final Hopper m_hopper = new Hopper();
  public static final ColorSensor m_colorSensor = new ColorSensor();
  private static RobotContainer m_robotContainer = new RobotContainer();
 // private static final Limelight m_limelight = new Limelight();
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final Command m_auto = new Auto(m_drivetrain, m_shooter);
  public final Command m_path = getAutonomousCommand("DriveToTarget");
   Timer m_timer = new Timer();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

/*
    Trajectory[] m_paths = new Trajectory[] {
      PathWeaver.getTrajectory("DriveToTarget")
    };
    */
    configureButtonBindings();
    m_chooser.setDefaultOption("PathWeaver", m_path);
    m_chooser.addOption("Autonomous", m_auto);
    // Put the chooser on the dashboard
    Shuffleboard.getTab("Autonomous").add(m_chooser);


//LeftY() should be negative because up on the left stick is a negative value

    m_drivetrain.setDefaultCommand(new RunCommand(() ->
    m_drivetrain.arcadeDrive(
      -m_driverController.getLeftY(), m_driverController.getRightX()), m_drivetrain));
  
  /*
    m_drivetrain.setDefaultCommand(new RunCommand(() -> 
    m_drivetrain.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY()), m_drivetrain));
*/
/*
m_drivetrain.setDefaultCommand(new RunCommand(() -> 
m_drivetrain.tankDriveVolts(-m_driverController.getLeftY() * 12, -m_driverController.getRightY() * 12), m_drivetrain));
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
        .whenPressed(new DriveDistance(100, m_drivetrain).withTimeout(3));
*/
        /*
        new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new SequentialCommandGroup(
          new RunCommand(() -> m_drivetrain.switchDriveMode("Brake"), m_drivetrain),
          new TurnToTarget(0, m_drivetrain),
          new RunCommand(() -> m_drivetrain.switchDriveMode("Coast"), m_drivetrain)));
          */
          
          new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new TurnToTarget(0, m_drivetrain).withTimeout(1));

      /*
        new JoystickButton(m_driverController, Button.kX.value)
        .whenPressed(new TurnToTargetProfiled(0, m_drivetrain));
        */
        new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(new SwitchDriveMode(m_drivetrain));
/*
        new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whenPressed(new RunCommand(() -> m_intake.setIntake(-.2), m_intake))
        .whenReleased(new RunCommand(() -> m_intake.setIntake(0), m_intake));
        *//*
        new JoystickButton(m_driverController, Button.kY.value)
        .whenPressed(new RunCommand(() -> m_drivetrain.baseDriveTo(100)));
        */
/*
        new JoystickButton(m_driverController, Button.kA.value)
        .whenPressed(
          new SequentialCommandGroup(
            new InstantCommand(() -> m_drivetrain.zeroHeading(), m_drivetrain),
          new TurnDegrees(-90, m_drivetrain).withTimeout(1.5)));
          */
/*
      new JoystickButton(m_driverController, Button.kY.value)
    .whenPressed(new Auto(m_drivetrain, m_shooter));
*/

    new JoystickButton(m_driverController, Button.kLeftBumper.value)
    .whenPressed(new RunCommand(() -> m_hopper.setHopper(HopperConstants.kHopperSpeed), m_hopper))
    .whenReleased(new RunCommand(() -> m_hopper.setHopper(0), m_hopper));

    new JoystickButton(m_driverController, Button.kRightBumper.value)
    .whenPressed(new RunCommand(() -> m_shooter.manualSpinMotor(ShooterConstants.kManualPwr), m_shooter))
    .whenReleased(new RunCommand(() -> m_shooter.manualSpinMotor(0), m_shooter));
    
    //Intake System
    new JoystickButton(m_driverController, Button.kB.value)
                        .whenPressed(() -> m_intake.retractIntake(), m_hopper)
                        .whileHeld(() -> {
                                m_intake.setIntake(IntakeConstants.kIntakeSpeed);
                                m_hopper.setHopper(HopperConstants.kHopperSpeed);
                        }, m_intake, m_hopper)
                        .whenReleased(() -> {
                                m_intake.setIntake(0);
                                m_intake.retractIntake();
                                m_hopper.setHopper(0);
                                
                        }, m_intake, m_hopper);
                        
  //Hopper System
  //May have to input solenoid boolean values, I don't think so
  new JoystickButton(m_driverController, Button.kX.value)
                        .whenPressed(() -> m_hopper.toggleSolenoid(), m_hopper)
                        .whileHeld(() -> m_hopper.setHopper(HopperConstants.kHopperSpeed), m_hopper)
                        .whenReleased(() -> {
                                m_hopper.setHopper(0);
                                m_hopper.toggleSolenoid();
                        }, m_hopper);
                //new JoystickButton(m_driverController, Button.kB.value)
                        //.whileHeld(() -> m_hopper.toggleSolenoid(), m_hopper);
                        

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */
  public static Command getAutonomousCommand(String path) {
    m_drivetrain.resetAllSensors();
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
            List.of(new Translation2d(2, 2), new Translation2d(3, -2)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(5, 0, new Rotation2d(0)),
            // Pass config
            config);
*/
        m_drivetrain.m_field.getObject("traj").setTrajectory(exampleTrajectory);

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
  public Command getAutonomousChooser() {
    return m_chooser.getSelected();
  }
}