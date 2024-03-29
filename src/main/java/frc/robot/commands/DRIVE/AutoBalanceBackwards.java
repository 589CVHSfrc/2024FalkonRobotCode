// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DRIVE;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoBalanceBackwards extends Command {
  /** Creates a new PigeonBalance. */
  private DriveSubsystem m_drive;
  private Timer m_timer;

  public AutoBalanceBackwards(DriveSubsystem drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_timer = new Timer();

    addRequirements(drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override

  public void execute() {
    // System.out.println(m_timer.get());
    m_drive.drive(-m_drive.pitchAdjustVelocity(), 0, 0, true, true);

    if (m_drive.pitchAdjustVelocity() == 0) {
      m_timer.start();

    } else {

      m_timer.stop();
      m_timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_drive.setX();
    m_timer.stop();
    m_timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_timer.hasElapsed(3)) {
      // m_drive.setX();
      return true;
    }

    else {
      return false;
    }
  }

}