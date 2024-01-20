// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ARM;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.SpinnersSubsystem;

// public class ShootIntakeFast extends Command {
//   /** Creates a new GrabArm. */
//   private SpinnersSubsystem m_spin;
//   public ShootIntakeFast(SpinnersSubsystem spin) {
//     m_spin = spin;
//     addRequirements(spin);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if(m_spin.isSlow()){
//       m_spin.spinSlow(Constants.ArmSpinConstants.kSpinningDirection);
//     }
//     else{
//       m_spin.spinFast(Constants.ArmSpinConstants.kSpinningDirection);
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(m_spin.isSlow()){
//       m_spin.spinSlow(Constants.ArmSpinConstants.kSpinningDirection);
//     }
//     else{
//       m_spin.spinFast(Constants.ArmSpinConstants.kSpinningDirection);
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     m_spin.stopSpin();
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
