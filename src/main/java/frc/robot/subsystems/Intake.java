// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public final CANSparkMax m_intakeMotor;
  //private final Solenoid m_intakeSolenoid;
  
  public Intake() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.intakePort, MotorType.kBrushless);
   // m_intakeSolenoid = new Solenoid(IntakeConstants.intakeSolenoidRight);
  }

  public void setIntake(double speed) {
    m_intakeMotor.set(speed);
  }


/*
  public void retractIntake() {
    m_intakeSolenoid.toggle();
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
