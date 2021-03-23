// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class TurnWinch extends CommandBase {
  private final ClimbSubsystem m_climb;
  private final double m_direction;

  /**
   * The Command to wind the winch aka raise the robot. Motor will stop at the end of the command.
   * @param direction The way the winch will wind. 1 for up. -1 for down.
   * @param climb The subsystem to use the motor.
   */
  public TurnWinch(double direction, ClimbSubsystem climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_direction = direction;
    m_climb = climb;
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.getWinchMotor().set(m_direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.getWinchMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
