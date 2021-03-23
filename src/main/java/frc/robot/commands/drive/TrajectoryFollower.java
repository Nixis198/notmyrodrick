// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryFollower extends RamseteCommand {
  private final DriveSubsystem m_drive;
  private final String m_trajectory;
  
  /**
   * Makes a RamseteCommand with a provided trajectory.
   * @param trajectory the trajectory the robot will follow.
   * @param drive the subsystem to contorl the motors.
   */
  public TrajectoryFollower(String trajectory, DriveSubsystem drive) {
    super(
    drive.getTrajectoryFromJSON(trajectory),
    drive::getPose,
    new RamseteController(
      TrajectoryConstants.kRamseteB,
      TrajectoryConstants.kRamseteZeta),
    new SimpleMotorFeedforward(
      TrajectoryConstants.ksVolts,
      TrajectoryConstants.kvVoltSecondsPerMeter,
      TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
    TrajectoryConstants.kDriveKinematics,
    drive::getWheelSpeed,
    new PIDController(
      TrajectoryConstants.kPDriveVel,0,0),
    new PIDController(
      TrajectoryConstants.kPDriveVel,0,0),
    drive::tankDriveVolts,
    drive);

    m_drive = drive;
    m_trajectory = trajectory;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetOdometry(m_drive.getTrajectoryFromJSON(m_trajectory).getInitialPose());
    super.initialize();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stopDrive();
    super.end(interrupted);
  }
}
