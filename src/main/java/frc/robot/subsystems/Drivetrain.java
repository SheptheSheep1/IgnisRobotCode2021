// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public final WPI_TalonFX m_leftMaster;

  public final WPI_TalonFX m_rightMaster;

  public final WPI_TalonFX m_leftSlave;

  public final WPI_TalonFX m_rightSlave;

  private final DifferentialDrive m_diffDrive;

  private final MotorControllerGroup m_right;
  private final MotorControllerGroup m_left;

  DifferentialDriveOdometry m_odometry;
  
  public final AHRS m_gyro;
  
  private boolean bool;
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

    //Invert left side changed due to Differential drive no longer automatically inverting controllers
    m_leftMaster.setInverted(false); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    m_leftSlave.setInverted(false);
    m_rightMaster.setInverted(true);
    m_rightSlave.setInverted(true);
    //May have to invert the motor controllers when using voltage based output
    //m_right.setInverted(false); // CHANGE THESE UNTIL ROBOT DRIVES FORWARD
    //m_left.setInverted(false);

    //Set mode to coast rather than brake to perserve motors
    m_rightMaster.setNeutralMode(NeutralMode.Brake);
    m_rightSlave.setNeutralMode(NeutralMode.Brake);
    m_leftMaster.setNeutralMode(NeutralMode.Brake);
    m_rightSlave.setNeutralMode(NeutralMode.Brake);
    
    //Configure encoder
    m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightSlave.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //configure encoder
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
    m_leftSlave.setSelectedSensorPosition(0);
    m_rightSlave.setSelectedSensorPosition(0);
    // deleted sensor polarity because it does not work

    // diffdrive assumes by default that right side must be negative- change to false for master/slave config
   // m_diffDrive.setRightSideInverted(false); // DO NOT CHANGE THIS

    // deadband: motors wont move if speed of motors is within deadband
    m_diffDrive.setDeadband(DriveConstants.kDeadband);
    m_gyro = new AHRS(SPI.Port.kMXP);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
   
    bool = true;
  }
 
  public void switchDriveMode() {
    bool = !bool;
    //mode.equals("Brake")
    if (bool == true) {
      m_rightMaster.setNeutralMode(NeutralMode.Brake);
      m_rightSlave.setNeutralMode(NeutralMode.Brake);
      m_leftMaster.setNeutralMode(NeutralMode.Brake);
      m_leftSlave.setNeutralMode(NeutralMode.Brake);
      System.out.println("Mode: Brake");
    } else {
      m_rightMaster.setNeutralMode(NeutralMode.Coast);
      m_rightSlave.setNeutralMode(NeutralMode.Coast);
      m_leftMaster.setNeutralMode(NeutralMode.Coast);
      m_leftSlave.setNeutralMode(NeutralMode.Coast);
      System.out.println("Mode: Coast");
    }
  }
  
  public void arcadeDrive(final double forward, final double turn) {
    SmartDashboard.putNumber("forward", forward * DriveConstants.kDriveSpeed);
    SmartDashboard.putNumber("turn", turn * DriveConstants.kTurnSpeed);
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

  public void rotateToAngle(double angle, double threshold) {
    m_gyro.reset();
    double error = (Limelight.getTx() + m_gyro.getAngle()) - m_gyro.getAngle();

    if (error > threshold) {
      m_diffDrive.arcadeDrive(0, error*.05);
    }
  }
  /*
  public void tankDriveWithPIDF(double rightVelocitySetpoint, double leftVelocitySetpoint) {
    m_left.setVoltage(feedforward.calculate(leftVelocitySetpoint) + left_pid.calculate(getLeftEncoderRate(), leftVelocitySetpoint));
    m_right.setVoltage(feedforward.calculate(rightVelocitySetpoint) + right_pid.calculate(getRightEncoderRate(), rightVelocitySetpoint));
  }
*/

public void tankDriveVolts(double leftVolts, double rightVolts) {
  m_left.setVoltage(leftVolts);
  m_right.setVoltage(rightVolts);
  m_diffDrive.feed();
}

public void tankDriveVoltsV2(double leftVolts, double rightVolts) {
  var batteryVoltage = RobotController.getBatteryVoltage();
  if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
    leftVolts *= batteryVoltage / 12.0;
    rightVolts *= batteryVoltage / 12.0;
  }
  m_left.setVoltage(leftVolts);
  m_right.setVoltage(rightVolts);
  m_diffDrive.feed();
}
	public double getLeftEncoderPosition() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_leftMaster.getSelectedSensorPosition() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get distance by multiplying rotations of wheel by circumference of wheel (pi * diameter in meters)
    double distance = wheelRotations * (Math.PI * (.1524));

		return distance;
	}

  public double getRightEncoderPosition() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_rightMaster.getSelectedSensorPosition() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get distance by multiplying rotations of wheel by circumference of wheel (pi * diameter in meters)
    double distance = wheelRotations * (Math.PI * (.1524));

		return distance;
  }
  public double getRightEncoderRate() {
		// get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_rightMaster.getSelectedSensorVelocity() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get velocity by multiplying rotations of wheel by circumference of wheel (pi * diameter in meters)
    double velocity = wheelRotations * ((Math.PI * (.1524)));

    // get distance per second by multiplying by 10; getSelectedSensorVelocity sends distance back per 100ms
    velocity *= 10;

    return velocity;
	}
  public double getLeftEncoderRate() {
    // get rotations of encoder by dividing encoder counts by counts per rotation
    double encoderRotations = m_leftMaster.getSelectedSensorVelocity() / 2048;

    // get rotations of wheel by diving rotations of encoder by gear ratio
    double wheelRotations = encoderRotations / 8.4;

    // get velocity by multiplying rotations of wheel by circumference of wheel (pi * diameter in meters)
    double velocity = wheelRotations * (Math.PI * (.1524));

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

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public void resetOdometry(Pose2d pose) {
    m_leftMaster.setSelectedSensorPosition(0);
    m_rightMaster.setSelectedSensorPosition(0);
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  //
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate()); // we are sending in meters/second
  }
  //degrees -180 to 180; left should be going positive
  public double getHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    //return m_gyro.getRotation2d().getDegrees();
  }
  public double countAngle() {
    zeroHeading();
    return -m_gyro.getAngle();
  }
  public double getAngle() {
    return -m_gyro.getAngle();
  }
  public void zeroHeading() {
    m_gyro.reset();
  }
  public double getTurnRate() {
    return m_gyro.getRate();
  }
  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
}
}
