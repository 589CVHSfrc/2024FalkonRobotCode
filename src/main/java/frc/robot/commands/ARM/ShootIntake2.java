// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.ARM;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.subsystems.SpinnersSubsystem;

// public class ShootIntake2 extends Command {
//   /** Creates a new GrabArm. */
//   private SpinnersSubsystem m_spin;
//   private DoubleSupplier m_trigger;
//   public ShootIntake2(SpinnersSubsystem spin, DoubleSupplier trigger) {
//     m_trigger = trigger;
//     m_spin = spin;
//     // addRequirements(spin);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     if(m_trigger.getAsDouble()>.1){
//       if(m_spin.isSlow()){
//         m_spin.spinSlow(Constants.ArmSpinConstants.kSpinningDirection);
//       }
//       else{
//         m_spin.spin(Constants.ArmSpinConstants.kSpinningDirection);
//       }
//     }
    
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(m_trigger.getAsDouble()>.1){
//       if(m_spin.isSlow()){
//         m_spin.spinSlow(Constants.ArmSpinConstants.kSpinningDirection);
//       }
//       else{
//         m_spin.spin(Constants.ArmSpinConstants.kSpinningDirection);
//       }
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
