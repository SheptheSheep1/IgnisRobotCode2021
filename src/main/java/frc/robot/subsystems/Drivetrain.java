// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final WPI_TalonFX m_leftMaster, m_rightMaster, m_leftSlave, m_rightSlave;
  //private final SupplyCurrentLimitConfiguration m_currentLimitConfig;

  private final DifferentialDrive m_diffDrive;

  private final MotorControllerGroup m_right;
  private final MotorControllerGroup m_left;

  DifferentialDriveOdometry m_odometry;
  private final AHRS m_gyro;
  //PIDController left_pid;
  //PIDController right_pid;

  //SimpleMotorFeedforward feedforward;
  public Drivetrain() {
    m_leftMaster = new WPI_TalonFX(DriveConstants.dtFrontLeftPort);
    m_rightMaster = new WPI_TalonFX(DriveConstants.dtFrontRightPort);
    m_leftSlave = new WPI_TalonFX(DriveConstants.dtBackLeftPort);
    m_rightSlave = new WPI_TalonFX(DriveConstants.dtBackRightPort);

     //left_pid = new PIDController(0, 0, 0);
     //right_pid = new PIDController(0, 0, 0);

     //feedforward = new SimpleMotorFeedforward(1, 0, 0);

        m_left = new MotorControllerGroup(m_leftMaster, m_leftSlave);
        m_right = new MotorControllerGroup(m_rightMaster, m_rightSlave);

    m_diffDrive = new DifferentialDrive(m_left, m_right);

    m_leftSlave.follow(m_leftMaster);
    m_rightSlave.follow(m_rightMaster);

    m_left.setInverted(true); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    m_right.setInverted(false);// CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    
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
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  
  }
 
  public void arcadeDrive(final double forward, final double turn) {
    m_diffDrive.arcadeDrive(forward * DriveConstants.kDriveSpeed, turn * DriveConstants.kTurnSpeed, true);
  }

  public void tankDrive(final double left, final double right) {
    m_diffDrive.tankDrive(left * DriveConstants.kDriveSpeed, right * DriveConstants.kDriveSpeed);
  }

  // CURVATURE DRIVE = 2 STICK + QUICK TURN BUTTON
  public void curvatureDrive(final double forward, final double turn, final boolean quickTurn) {
    m_diffDrive.curvatureDrive(forward * DriveConstants.cDriveSpeed, turn * DriveConstants.cTurnSpeed, quickTurn);
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
  /*
  public void tankDriveWithPIDF(double rightVelocitySetpoint, double leftVelocitySetpoint) {
    m_left.setVoltage(feedforward.calculate(leftVelocitySetpoint) + left_pid.calculate(getLeftEncoderRate(), leftVelocitySetpoint));
    m_right.setVoltage(feedforward.calculate(rightVelocitySetpoint) + right_pid.calculate(getRightEncoderRate(), rightVelocitySetpoint));
  }
*/
public void tankDriveVolts(double leftVolts, double rightVolts) {
  m_leftMaster.setVoltage(leftVolts);
  m_rightMaster.setVoltage(rightVolts);
}

	public double getLeftEncoderPosition() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_leftMaster.getSelectedSensorPosition() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get distance by multiplying rotations of wh2.5 * 39.37eel by circumference of wheel (2 * pi * radius)
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
  public double getRightEncoderRate() {
		// get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_rightMaster.getSelectedSensorVelocity() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get velocity by multiplying rotations of wheel by circumference of wheel (2 * pi * radius)
    double velocity = wheelRotations * (2 * Math.PI * (2.5 * 39.37));

    // get distance per second by multiplying by 10; getSelectedSensorVelocity sends distance back per 100ms
    velocity *= 10;

    return velocity;
	}
  public double getLeftEncoderRate() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_leftMaster.getSelectedSensorVelocity() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get velocity by multiplying rotations of wheel by circumference of wheel (2 * pi * radius)
    double velocity = wheelRotations * (2 * Math.PI * (2.5 * 39.37));

    // get distance per second by multiplying by 10; getSelectedSensorVelocity sends distance back per 100ms
    velocity *= 10;

		return velocity;
	}
  public double getAverageEncoderRate() {
    return (double) ((getLeftEncoderRate() + getRightEncoderRate()) / 2.0);
  }
  public double getAverageEncoderDistance() {
    return (double) ((getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate()); // we are sending in meters/second
  }
}
