// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonFX m_leftMaster, m_rightMaster, m_leftSlave, m_rightSlave;
  private final XboxController m_driverController = new XboxController(0);
  //private final SupplyCurrentLimitConfiguration m_currentLimitConfig;

  private final DifferentialDrive m_diffDrive;

  private final MotorControllerGroup m_right;
  private final MotorControllerGroup m_left;

  private final Encoder encoder;

  //private boolean dtIsInverted;

  public Drivetrain() {
    m_leftMaster = new WPI_TalonFX(DriveConstants.dtFrontLeftPort);
    m_rightMaster = new WPI_TalonFX(DriveConstants.dtFrontRightPort);
    m_leftSlave = new WPI_TalonFX(DriveConstants.dtBackLeftPort);
    m_rightSlave = new WPI_TalonFX(DriveConstants.dtBackRightPort);

        m_left = new MotorControllerGroup(m_leftMaster, m_leftSlave);
        m_right = new MotorControllerGroup(m_rightMaster, m_rightSlave);

    m_diffDrive = new DifferentialDrive(m_left, m_right);

    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    m_left.setInverted(true); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    m_right.setInverted(false); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    
    encoder = new Encoder(1, 2);
    //dtIsInverted = false;

    // flip so that motor output and sensor velocity are same polarity
    m_leftMaster.setSensorPhase(false);
    m_rightMaster.setSensorPhase(false);
    m_leftSlave.setSensorPhase(false);
    m_rightSlave.setSensorPhase(false);
    
    // set mode of motors
    

    // diffdrive assumes by default that right side must be negative- change to false for master/slave config
   // m_diffDrive.setRightSideInverted(false); // DO NOT CHANGE THIS

    // deadband: motors wont move if speed of motors is within deadband
    m_diffDrive.setDeadband(DriveConstants.kDeadband);

  
  }
 
  public void arcadeDrive(final double forward, final double turn) {
    m_diffDrive.arcadeDrive(forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, true);
  }

  // CURVATURE DRIVE = 2 STICK + QUICK TURN BUTTON
  public void curvatureDrive(final double forward, final double turn, final boolean quickTurn) {
    m_diffDrive.curvatureDrive(forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, quickTurn);
  }

  public void baseDriveTo(double distance) {
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
    if (Limelight.calcDistance() > distance) {
      while (getAverageEncoderDistance() < distance ) {
        m_diffDrive.arcadeDrive(.2, 0);
      }
    } else if (Limelight.calcDistance() < distance) {
      while (getAverageEncoderDistance() > distance) {
        m_diffDrive.arcadeDrive(-.2, 0);
      }
    }
  }

	public double getLeftEncoderPosition() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_leftMaster.getSelectedSensorPosition() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get distance by multiplying rotations of wheel by circumference of wheel (2 * pi * radius)
    double distance = wheelRotations * (2 * Math.PI * (2.5 * 39.37));

		return distance;
	}

  public double getRightEncoderPosition() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_rightMaster.getSelectedSensorPosition() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get distance by multiplying rotations of wheel by circumference of wheel (2 * pi * radius)
    double distance = wheelRotations * (2 * Math.PI * (2.5 * 39.37));

		return distance;
  }
  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /*
  public boolean turnToTarget(Drivetrain drivetrain) {
    updateLimelightTracking();
    
    double isNegative;
    if (tx < 0) {
      isNegative = -1;
    } else {
      isNegative = 1;
    }

    double txRatio = Math.abs(tx / 90);
    txRatio += 0.1;
    txRatio *= isNegative;

    double m_rotationError = txRatio;
    m_steerAdjust = m_rotationError;

    if (m_limelightHasValidTarget) { // if limelight sees target
      if (Math.abs(tx) <= VisionConstants.kTargetZone) { // if target is within zone
        drivetrain.autoDrive(0.0, 0.0);
        return true;
      }
      drivetrain.autoDrive(0, m_steerAdjust); // drive using command-tuned values
      return false;
    } else {
      drivetrain.autoDrive(0.0, 0.0); // otherwise do nothing
      return true;
    }
  }

  */
}
