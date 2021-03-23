// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.music.Orchestra;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  // Motors on the Right Side
  private final WPI_TalonFX m_rightFront = new WPI_TalonFX(DriveConstants.kRightMotor1Port);
  private final WPI_TalonFX m_rightBack = new WPI_TalonFX(DriveConstants.kRightMotor2Port);

  // Motors on the Left Side
  private final WPI_TalonFX m_leftFront = new WPI_TalonFX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonFX m_leftBack = new WPI_TalonFX(DriveConstants.kLeftMotor2Port);

  // Groups for the motors
  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(m_rightFront, m_rightBack);
  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(m_leftFront, m_leftBack);

  // The navX Gryo
  private final AHRS m_navX = new AHRS(SPI.Port.kMXP);

  // The Robot's Drives
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_navX.getRotation2d());

  // The Orchestra to make the robot sing
  private final Orchestra m_orchestra = new Orchestra();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_rightFront.configFactoryDefault();
    m_rightBack.configFactoryDefault();
    m_leftFront.configFactoryDefault();
    m_leftBack.configFactoryDefault();

    m_rightFront.setInverted(false);
    m_rightBack.setInverted(false);
    m_leftFront.setInverted(false);
    m_leftBack.setInverted(false);

    m_rightFront.setNeutralMode(NeutralMode.Brake);
    m_rightBack.setNeutralMode(NeutralMode.Brake);
    m_leftFront.setNeutralMode(NeutralMode.Brake);
    m_leftBack.setNeutralMode(NeutralMode.Brake);

    m_rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_rightBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_leftBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_drive.setRightSideInverted(true);

    m_orchestra.addInstrument(m_rightFront);
    m_orchestra.addInstrument(m_rightBack);
    m_orchestra.addInstrument(m_leftFront);
    m_orchestra.addInstrument(m_leftBack);
  }

/**
 * The Method for the arcade drive command.
 * @param xSpeed How fast the robot will drive.
 * @param zRotation How fast the robot will rotate.
 */
  public void arcadeDrive(double xSpeed, double zRotation) {
    m_drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * A flipped arcadeDrive method. It flips which side the front the robot is.
   * @param xSpeed How fast the robot will drive.
   * @param zRotation How fast the robot will rotate.
   */
  public void flipArcadeDrive(double xSpeed, double zRotation) {
    m_drive.arcadeDrive(-xSpeed, zRotation);
  }

  /**
   * Controls the right and left sides of the drives directly with voltages.
   * @param rightVolts the commanded right output.
   * @param leftVolts the commanded left output.
   */
  public void tankDriveVolts(double rightVolts, double leftVolts) {
    m_rightMotors.setVoltage(rightVolts);
    m_leftMotors.setVoltage(leftVolts);
    m_drive.feed();
  }

  /**
   * Will stop the robot from driving.
   */
  public void stopDrive(){
    m_drive.stopMotor();
  }

  /**
   * Sets the max speed of the drive.
   * @param limit How fast do you want to limit the robot's speed.
   */
  public void setMaxSpeed(double limit) {
    m_drive.setMaxOutput(limit);
  }

  /**
   * Gets the raw velocity from both right motors and averages them together before turning them into meters per second.
   * @return How fast the right side is going in meters per second.
   */
  public double getRightVelocity() {
    return ((m_rightFront.getSelectedSensorVelocity() + m_rightBack.getSelectedSensorVelocity()) / 2) * DriveConstants.kMetersPerTick;
  }

  /**
   * Gets the raw velocity from both left motors and averages them together before turning them into meters per second.
   * @return How fast the left side is going in meters per second.
   */
  public double getLeftVelocity() {
    return ((m_leftFront.getSelectedSensorVelocity() + m_leftBack.getSelectedSensorVelocity()) / 2) * DriveConstants.kMetersPerTick;
  }

  /**
   * Gets the raw distance from bot the right motors and averages them together before turning them into distance in meters.
   * @return The distance the right side has gone.
   */
  public double getRightDistanceMeters() {
    return ((((m_rightFront.getSelectedSensorPosition() + m_rightBack.getSelectedSensorPosition()) / 2) / DriveConstants.kEncoderCPR) / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumferenceMeters;
  }

  /**
   * Gets the raw distance from bot the left motors and averages them together before turning them into distance in meters.
   * @return The distance the left side has gone.
   */
  public double getLeftDistanceMeters() {
    return ((((m_leftFront.getSelectedSensorPosition() + m_leftBack.getSelectedSensorPosition()) / 2) / DriveConstants.kEncoderCPR) / DriveConstants.kGearRatio) * DriveConstants.kWheelCircumferenceMeters;
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The curretnt wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeed() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from -180 to 180.
   */
  public double getHeading() {
    return m_navX.getRotation2d().getDegrees();
  }

  /**
   * Returns the currently estimated pose of the robot.
   * @return The Pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets all the motor encoders back to 0.
   */
  public void resetEncoders() {
    m_rightFront.setSelectedSensorPosition(0);
    m_rightBack.setSelectedSensorPosition(0);
    m_leftFront.setSelectedSensorPosition(0);
    m_leftBack.setSelectedSensorPosition(0);
  }

  /**
   * Resets the odometry to the specified pose.
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_navX.getRotation2d());
  }

  /**
   * Zeros the heading of the robot.
   */
  public void zeroHeading() {
    m_navX.reset();
  }

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second.
   */
  public double getTurnRate() {
    return -m_navX.getRate();
  }

  /**
   * Passes the orchestra object to use.
   * @return the orchestra to the command.
   */
  public Orchestra getOrchestra(){
    return m_orchestra;
  }

  /**
   * Reads the trajectory json file and returns it as a trajectory object.
   * @param name The name of the json file.
   * @return The trajectory for the robot to follow.
   */
  public Trajectory getTrajectoryFromJSON(String name) {
    String path = "output/" + name + ".wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      DriverStation.reportError("Unable to open Trajectory: " + path, e.getStackTrace());
    }
    return trajectory;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update the odometry in the periodic block
    m_odometry.update(m_navX.getRotation2d(), getLeftDistanceMeters(), getRightDistanceMeters());
  }
}
