// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.AUTO;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.commands.ARM.RaiseArm;
// import frc.robot.commands.ARM.ShootIntake;
// import frc.robot.commands.ARM.ShootIntakeTimed;
// import frc.robot.commands.ARM.ShootIntakeTimed2;
// import frc.robot.commands.DRIVE.AutoAlign;
// import frc.robot.commands.DRIVE.AutoBalance;
// import frc.robot.commands.DRIVE.DriveForward;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.SpinnersSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ScoreTaxi extends SequentialCommandGroup {
//   /** Creates a new ScoreBalance. */
//   public ScoreTaxi(DriveSubsystem drive, SpinnersSubsystem spin) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());
//     addCommands(
//       //  new AutoAlign(drive),
//        new ShootIntakeTimed2(spin),
//        new WaitCommand(2),
//        new DriveForward(-.2, 4, drive)
//       //  new DriveForward(-1, 1, drive),
//       //  new AutoBalance(drive)
//     );
//   }
// }
