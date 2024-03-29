// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ARM;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.ArmSubsystem;

// public class LowerArm extends Command {
//   /** Creates a new RaiseArm. */
//   private ArmSubsystem m_arm;
//   private double m_speed;

//   public LowerArm(ArmSubsystem arm, double speed) {
//     m_arm = arm;
//     m_speed = speed;
//     addRequirements(arm);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_arm.moveArm(() -> -m_speed);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     m_arm.moveArm(() -> -m_speed);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_arm.stopArm();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return m_arm.getCurrentAngle() >= 0.7 ;
//   }
// }
