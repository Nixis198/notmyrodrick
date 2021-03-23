// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  // Makes the intake Motor
  private final WPI_VictorSPX m_intakeMotor = new WPI_VictorSPX(IntakeConstants.kIntakeMotorPort);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Setting up the motor
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.setInverted(true);
    m_intakeMotor.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Gives the motor so the command can use it.
   * @return motor to the command.
   */
  public WPI_VictorSPX getIntakeMotor() {
    return m_intakeMotor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
