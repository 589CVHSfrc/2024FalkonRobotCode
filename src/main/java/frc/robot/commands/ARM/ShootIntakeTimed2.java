// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ARM;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.SpinnersSubsystem;

// public class ShootIntakeTimed2 extends Command {
//   /** Creates a new GrabArm. */
//   private SpinnersSubsystem m_spin;
//   private Timer m_timer;
//   public ShootIntakeTimed2(SpinnersSubsystem spin) {
//     m_spin = spin;
//     m_timer = new Timer();
//     addRequirements(spin);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_timer.start();
//     if(m_spin.isSlow()){
//       m_spin.spinSlow(Constants.ArmSpinConstants.kSpinningDirection);
//     }
//     else{
//       m_spin.spinHalf(Constants.ArmSpinConstants.kSpinningDirection);
//     }
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(m_spin.isSlow()){
//       m_spin.spinSlow(Constants.ArmSpinConstants.kSpinningDirection);
//     }
//     else{
//       m_spin.spinHalf(Constants.ArmSpinConstants.kSpinningDirection);
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
//     return m_timer.get()>.5;
//   }
// }
