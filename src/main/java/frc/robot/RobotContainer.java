// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.R2Jesu_DriveCommand;
import frc.robot.commands.R2Jesu_ForwardHangCommand;
import frc.robot.commands.R2Jesu_LightShotCommand;
import frc.robot.commands.R2Jesu_ReverseHangCommand;
import frc.robot.commands.R2Jesu_ReverseIntakeCommand;
import frc.robot.commands.R2Jesu_ShootCommand;
import frc.robot.commands.R2Jesu_ShooterCommand;
import frc.robot.subsystems.R2Jesu_DriveSubsystem;
import frc.robot.subsystems.R2Jesu_HangerSubsystem;
import frc.robot.subsystems.R2Jesu_ShooterSubsystem;
 

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  AHRS ahrs = new AHRS(AHRS.NavXComType.kMXP_SPI);
  R2Jesu_Limelight robotLimelight = new R2Jesu_Limelight();
  private final R2Jesu_DriveSubsystem m_R2Jesu_DriveSubsystem = new R2Jesu_DriveSubsystem(ahrs);
  private final R2Jesu_ShooterSubsystem m_R2Jesu_ShooterSubsystem = new R2Jesu_ShooterSubsystem(robotLimelight);
  private final R2Jesu_HangerSubsystem m_R2Jesu_HangerSubsystem = new R2Jesu_HangerSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_R2Jesu_DriveSubsystem.setDefaultCommand(
     new R2Jesu_DriveCommand(m_R2Jesu_DriveSubsystem, 
        () -> m_driverController.getRightX(),
        () ->  m_driverController.getRightY(),
        () -> -m_driverController.getLeftX()));
    
    m_R2Jesu_ShooterSubsystem.setDefaultCommand(
     new R2Jesu_ShooterCommand(m_R2Jesu_ShooterSubsystem)); 
     
   m_R2Jesu_HangerSubsystem.setDefaultCommand(
     new R2Jesu_ReverseHangCommand(m_R2Jesu_HangerSubsystem)); 
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `R2Jesu_DriveCommand` when `R2Jesu_DriveCondition` changes to `true`
    new JoystickButton(m_driverController, XboxController.Button.kA.value).whileTrue(new R2Jesu_LightShotCommand(m_R2Jesu_ShooterSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kB.value).whileTrue(new R2Jesu_ShootCommand(m_R2Jesu_ShooterSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kX.value).whileTrue(new R2Jesu_ReverseIntakeCommand(m_R2Jesu_ShooterSubsystem));
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(new R2Jesu_ForwardHangCommand(m_R2Jesu_HangerSubsystem));

    // Schedule `R2Jesu_DriveMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_R2Jesu_DriveSubsystem.R2Jesu_DriveMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An R2Jesu_Drive command will be run in autonomous
    //return Autos.R2Jesu_DriveAuto(m_R2Jesu_DriveSubsystem);
  //}
}
