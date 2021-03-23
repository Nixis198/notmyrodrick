// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class TelescopeUp extends CommandBase {
  private final ClimbSubsystem m_climb;

  /**
   * The command to move the telescope up.
   * @param climb The subsystem needed to control the motor.
   */
  public TelescopeUp(ClimbSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climb = climb;
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.getTelescopeMotor().set(ClimbConstants.kTelescopeSpeedUp);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.getTelescopeMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_climb.getRotations()) >= ClimbConstants.kRotationsToTop;
  }
}
