// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.R2Jesu_DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An R2Jesu_Drive command that uses an R2Jesu_Drive subsystem. */
public class R2Jesu_DriveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final R2Jesu_DriveSubsystem m_subsystem;
  private DoubleSupplier m_x;
  private DoubleSupplier m_y;
  private DoubleSupplier m_z;

  /**
   * Creates a new R2Jesu_DriveCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public R2Jesu_DriveCommand(R2Jesu_DriveSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z) {
    m_subsystem = subsystem;
    m_x = x;
    m_y = y;
    m_z = z;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.drive(m_x.getAsDouble(), m_y.getAsDouble(), m_z.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
