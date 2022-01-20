// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public final CANSparkMax m_master;
private final CANSparkMax m_slave;

  public Shooter() {
    m_master = new CANSparkMax(ShooterConstants.shooterFirstPort, MotorType.kBrushless);
    m_slave = new CANSparkMax(ShooterConstants.shooterSecondPort, MotorType.kBrushless);

    m_slave.follow(m_master, true);

    m_master.getEncoder().setPosition(0);
    m_slave.getEncoder().setPosition(0);
  }

  public void manualSpinMotor(double speed) {
    m_master.set(speed);
  }

  public double getSpeed() {
    return m_master.getEncoder().getVelocity() / ShooterConstants.kNeoRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
