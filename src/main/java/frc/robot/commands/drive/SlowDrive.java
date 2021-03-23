// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SlowDrive extends CommandBase {
  private final DriveSubsystem m_drive;
  private final double m_limit;

  /**
   * Will slow down the max speed of the robot by the sent limit. Will return it back to 100% when the command is over.
   * @param limit The max speed that the robot will drive.
   * @param drive The subsystem to control the motors.
   */
  public SlowDrive(double limit, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_limit = limit;
    m_drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.setMaxSpeed(m_limit);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.setMaxSpeed(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
