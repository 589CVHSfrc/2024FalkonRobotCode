// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.auto.PIDConstants;
// import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DRIVE.AutoBalanceBackwards;
import frc.robot.commands.DRIVE.DefaultDrive;
import frc.robot.commands.DRIVE.ResetGyro;
// import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.SpinnersSubsystem;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final static DriveSubsystem m_robotDrive = new DriveSubsystem();
        // private final static SpinnersSubsystem m_robotSpin = new SpinnersSubsystem();
        // private final static ArmSubsystem m_robotArm = new ArmSubsystem();
        // private final Gyro m_Gyro = new Gyro();
        // The driver's controller
        XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
        // XboxController m_codriverController = new
        // XboxController(OIConstants.kCODriverControllerPort);
        private SendableChooser<Command> m_autoChooser = new SendableChooser<>();
        private final Pose2d m_zero = new Pose2d();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                // Command test = m_autoBuilder.fullAuto(new PathPlannerTrajectory());

                // m_autoChooser.addOption("Score TAXIIII", new ScoreTaxi(m_robotDrive,
                // m_robotSpin));
                // m_autoChooser.addOption("Score BALANCE", new ScoreBal(m_robotDrive,
                // m_robotSpin));
                // m_autoChooser.addOption("Score ONLY", new ScoreONLY(m_robotDrive,
                // m_robotSpin));
                // m_autoChooser.addOption("Score Backup Balance", new
                // ScoreBalance(m_robotDrive, m_robotSpin));
                // m_autoChooser.setDefaultOption("NOTHING", new NOTHING());

                SmartDashboard.putData(m_autoChooser);

                // Configure the button bindings
                configureButtonBindings();
                m_robotDrive.configureHolonomicAutoBuilder();
                m_autoChooser.setDefaultOption("Donuts", new PathPlannerAuto("Donuts"));
                m_autoChooser.addOption("POS NEG TEST AUTO", new PathPlannerAuto("PosNegTestAuto"));
                // m_autoChooser.addOption("Donuts", new PathPlannerAuto("Donuts"));
                // m_autoChooser.addOption("Auto1", new PathPlannerAuto("New Auto"));

                // Configure default commands
                // m_robotArm.setDefaultCommand(
                // new RunCommand(
                // () -> m_robotArm.moveArm(
                // () -> 0),

                // m_robotArm));
                m_robotDrive.setDefaultCommand(new DefaultDrive(m_robotDrive,
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                OIConstants.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                OIConstants.kDriveDeadband),
                                () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(2),
                                                OIConstants.kDriveDeadband)));
        }

        private void configureButtonBindings() {

                new JoystickButton(m_driverController, 4)
                                .toggleOnTrue(new ResetGyro(m_robotDrive));
                new JoystickButton(m_driverController, 1)
                                .toggleOnTrue(new AutoBalanceBackwards(m_robotDrive));

                new JoystickButton(m_driverController, 2)
                                .whileTrue(new RunCommand(
                                                () -> m_robotDrive.setX(),
                                                m_robotDrive));

        }

        public Command getAutonomousCommand() {

                // m_robotDrive.configureHolonomicAutoBuilder();
                // return m_autoChooser.getSelected();
                // return null;
                try {

                        Pose2d startingpose = PathPlannerAuto
                                        .getStaringPoseFromAutoFile(m_autoChooser.getSelected().getName());
                        m_robotDrive.resetOdometry(startingpose);
                        System.out.print("====================POSE: " + startingpose + "==============");
                        return m_autoChooser.getSelected();
                        // return new PathPlannerAuto("Testing");

                } catch (RuntimeException e) {
                        System.out.print("==================" + e);
                        System.out.print("===COULD NOT FIND AUTO WITH SELECTED NAME===");
                        return new WaitCommand(1);
                }

        }

}
