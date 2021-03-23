// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
  private final IntakeSubsystem m_intake;
  private final double m_direction;

  /**
   * Command to run the intake. Can suck in balls or spit them out. Will stop the motor after command is done.
   * @param direction Which way the motor will spin. 1 for intake. -1 for output.
   * @param intake The subsystem needed to control the motors.
   */
  public RunIntake(double direction, IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_direction = direction;
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.getIntakeMotor().set(m_direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.getIntakeMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
