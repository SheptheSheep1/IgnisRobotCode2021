// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UnitConversionConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public final CANSparkMax m_master;
private final CANSparkMax m_slave;
  private final Limelight m_limelight;

  public Shooter() {
    m_master = new CANSparkMax(ShooterConstants.shooterFirstPort, MotorType.kBrushless);
    m_slave = new CANSparkMax(ShooterConstants.shooterSecondPort, MotorType.kBrushless);
    m_limelight = new Limelight();
    m_slave.follow(m_master, true);

    m_master.getEncoder().setPosition(0);
    m_slave.getEncoder().setPosition(0);
  }

  public void manualSpinMotor(double speed) {
    m_master.set(speed);
  }
  public double calcVeloPerDistance(double distance) {
    double o = ShooterConstants.kShooterAngle * UnitConversionConstants.angleConversionFactor;
    double g = PhysicsConstants.gAcceleration;
    double d = distance + FieldConstants.kOuterToInnerTarget;
    double hI = ShooterConstants.kShooterHeight;
    double hD = Limelight.calcDistance();
    double velocity = (((distance * Math.sqrt(386.89 * 0.5)) / (Math.cos(o) * Math.sqrt(Math.tan(o)*d + (hI - hD)))) + .01);
    // velocity
    velocity = (velocity * 60) / (5880.0 * 1.75 * 2.0 * Math.PI * ShooterConstants.kMotorRadius);
    return velocity;
  }
  
  public double getSpeed() {
    return m_master.getEncoder().getVelocity() / ShooterConstants.kNeoRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
