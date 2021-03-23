// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.climb.TelescopeDown;
import frc.robot.commands.climb.TelescopeUp;
import frc.robot.commands.climb.TurnWinch;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.ReverseDrive;
import frc.robot.commands.drive.SlowDrive;
import frc.robot.commands.drive.TrajectoryFollower;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.pneumatics.OpenGate;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final PneumaticsSubsystem m_blow = new PneumaticsSubsystem();
  private final ClimbSubsystem m_climb = new ClimbSubsystem();

  // Controllers
  //private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  private final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

  // Auto Commands
  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();
  private final TrajectoryFollower m_testPath = new TrajectoryFollower("TestPath", m_drive);
  private final TrajectoryFollower m_yeetPath = new TrajectoryFollower("Yeet", m_drive);
  private final TrajectoryFollower m_barrel = new TrajectoryFollower("Barrel", m_drive);

  // Shuffleboard settings
  private final ShuffleboardTab m_mainTab = Shuffleboard.getTab("Main Tab");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure the default commands for all the subsystems
    m_drive.setDefaultCommand(new ArcadeDrive(() -> -m_driverController.getY(GenericHID.Hand.kLeft), () -> m_driverController.getX(GenericHID.Hand.kLeft), m_drive));

    // Making the camera
    CameraServer.getInstance().startAutomaticCapture(0);

    m_mainTab.add(m_autoChooser).withSize(2, 1);
    m_autoChooser.addOption("Test Path", m_testPath);
    m_autoChooser.addOption("Yeet", m_yeetPath);
    m_autoChooser.addOption("Barrel", m_barrel);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Controls */
    new JoystickButton(m_driverController, Button.kBumperLeft.value).whenHeld(new SlowDrive(OIConstants.kSlowDriveLimit, m_drive)); // Slows the robot down when held
    new JoystickButton(m_driverController, Button.kBack.value).whenHeld(new TurnWinch(-1, m_climb));                                // Unwind the winch
    new JoystickButton(m_driverController, Button.kBumperRight.value).whenHeld(new ReverseDrive(() -> m_driverController.getY(GenericHID.Hand.kLeft), () -> m_driverController.getX(GenericHID.Hand.kLeft), m_drive)); // Flips which side the front is

    /* Operator Controls */
    new JoystickButton(m_operatorController, 5).whenHeld(new RunIntake(1, m_intake));    // Runs the intake to suck up balls
    new JoystickButton(m_operatorController, 6).whenHeld(new RunIntake(-1, m_intake));   // Runs the intake to spit out balls
    new JoystickButton(m_operatorController, 1).whenHeld(new OpenGate(m_blow));          // Opens the gate to release the balls
    new JoystickButton(m_operatorController, 9).whenPressed(new TelescopeUp(m_climb));   // Raises the telescope up
    new JoystickButton(m_operatorController, 8).whenPressed(new TelescopeDown(m_climb)); // Lowers the telescope down
    new JoystickButton(m_operatorController, 16).whenHeld(new TurnWinch(1, m_climb));    // Raises the robot
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoChooser.getSelected();
  }
}
