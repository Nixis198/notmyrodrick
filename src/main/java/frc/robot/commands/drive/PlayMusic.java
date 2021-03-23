// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PlayMusic extends CommandBase {
  private final DriveSubsystem m_drive;
  private final String m_song;

  /**
   * Plays the selected song with the talons.
   * @param song Song to be played.
   * @param drive The subsystem to use the motors.
   */
  public PlayMusic(String song, DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_song = song;
    m_drive = drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.getOrchestra().loadMusic("songs/" + m_song + ".chrp");
    m_drive.getOrchestra().play();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.getOrchestra().stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_drive.getOrchestra().isPlaying();
  }
}
