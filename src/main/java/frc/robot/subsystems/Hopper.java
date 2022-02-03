// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  public final CANSparkMax m_hopperMotor;
  //private final Solenoid m_hopperPiston;

  public Hopper() {
    m_hopperMotor = new CANSparkMax(12, MotorType.kBrushless);
    //m_hopperPiston = new Solenoid(HopperConstants.hopperSolenoid);
  }

  public void setHopper(double speed) {
    m_hopperMotor.set(speed);
  }

  // using a single solenoid as double solenoid
  /*
  public void toggleSolenoid() {
    m_hopperPiston.toggle();
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
