// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class PneumaticsSubsystem extends SubsystemBase {
  // Makes the Compressor
  private final Compressor m_compressor = new Compressor();

  // Makes the Solenoids
  private final Solenoid m_gateSolenoid = new Solenoid(PneumaticsConstants.kGateSolenoidPort);

  /** Creates a new PneumaticsSubsystem. */
  public PneumaticsSubsystem() {
    // Sets up the pneumatics
    m_compressor.setClosedLoopControl(true);
  }

  /**
   * Passes the solenoid to the command.
   * @return the solenoid to use.
   */
  public Solenoid getGateSolenoid() {
    return m_gateSolenoid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
