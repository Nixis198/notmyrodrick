// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  // Makes the winch motor
  private final WPI_VictorSPX m_winchMotor = new WPI_VictorSPX(ClimbConstants.kWinchMotor);

  // Makes the telescope motor
  private final WPI_TalonSRX m_telescopeMotor = new WPI_TalonSRX(ClimbConstants.kTelescopeMotor);

  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    // Sets up the motors
    m_winchMotor.configFactoryDefault();
    m_telescopeMotor.configFactoryDefault();

    m_winchMotor.setInverted(true);
    m_telescopeMotor.setInverted(false);

    m_winchMotor.setNeutralMode(NeutralMode.Brake);
    m_telescopeMotor.setNeutralMode(NeutralMode.Brake);

    // Sets up the encoder
    m_telescopeMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }

  /**
   * Passes the motor to the command.
   * @return the motor to use.
   */
  public WPI_VictorSPX getWinchMotor() {
    return m_winchMotor;
  }

  /**
   * Passes the motor to the command.
   * @return the motor to use.
   */
  public WPI_TalonSRX getTelescopeMotor() {
    return m_telescopeMotor;
  }

  /**
   * Resets the telescope's encoder back to 0.
   */
  public void resetEncoder() {
    m_telescopeMotor.setSelectedSensorPosition(0);
  }

  /**
   * Calculates how many times the telescope motor has turned.
   * @return the number of rotations.
   */
  public double getRotations() {
    return m_telescopeMotor.getSelectedSensorPosition() / ClimbConstants.kEncoderCPR;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
