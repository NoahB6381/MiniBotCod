package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Arm.Intake.ArmIntakeInCmd;
import frc.robot.commands.Arm.Intake.ArmIntakeOutCmd;
import frc.robot.commands.Arm.Manipulator.ArmManipulatorDriveCmd;
import frc.robot.commands.Arm.Manipulator.ArmManipulatorIntakeCmd;
import frc.robot.commands.Auto.AutoWaitCmd;
import frc.robot.commands.Auto.Movement.AutoChargingBalanceCmd;
import frc.robot.commands.Auto.Movement.AutoDriveCmd;
import frc.robot.commands.Auto.Movement.Trajectories;
import frc.robot.commands.Drive.Allign.DriveAllignPoleCmd;
import frc.robot.commands.Drive.Default.SwerveJoystickCmd;
import frc.robot.commands.Drive.Gyro.DriveGyroResetCmd;
import frc.robot.subsystems.Primary.SwerveSubsystem;
import frc.robot.subsystems.Secondary.ArmSubsystem;
import frc.robot.subsystems.Secondary.RotateSubsystem;

public class RobotContainer {

        // private final Command autoMiddle = new SequentialCommandGroup(
        //                 new InstantCommand(
        //                                 () -> swerveSubsystem.resetOdometry(Trajectories.getTraj1().getInitialPose())),
        //                 Trajectories.traj3(), // move to charge station
        //                 Trajectories.traj1(), // move then stop
        //                 // new AutoWaitCmd(500), // stop
        //                 Trajectories.traj2(), // go on the drive station
        //                 Commands.race(new AutoDriveCmd(swerveSubsystem, 0.5), new AutoWaitCmd(10)),
        //                 new AutoChargingBalanceCmd(swerveSubsystem),
        //                 // new Gyro180Cmd(swerveSubsystem),
        //                 new InstantCommand(() -> swerveSubsystem.stopModules()));

        // private final Command autoSide = new SequentialCommandGroup(
        //                 new InstantCommand(
        //                                 () -> swerveSubsystem.resetOdometry(Trajectories.getTraj4().getInitialPose())),
        //                 // new Gyro180Cmd(swerveSubsystem),
        //                 Trajectories.traj4(),
        //                 // new DriveGyro180Cmd(swerveSubsystem),
        //                 new InstantCommand(() -> swerveSubsystem.stopModules()));

        // private final Command autoSideLeft = new SequentialCommandGroup(
        //                 new InstantCommand(
        //                                 () -> swerveSubsystem.resetOdometry(Trajectories.getTraj6().getInitialPose())),
        //                 // new Gyro180Cmd(swerveSubsystem),
        //                 Trajectories.traj4(),
        //                 // new DriveGyro180Cmd(swerveSubsystem),
        //                 new InstantCommand(() -> swerveSubsystem.stopModules()));

        // private final Command autoPlace = new SequentialCommandGroup(
        //                 // new Gyro180Cmd(swerveSubsystem),
        //                 new InstantCommand(
        //                                 () -> swerveSubsystem.resetOdometry(Trajectories.getTraj1().getInitialPose())),
        //                 // new DriveGyro180Cmd(swerveSubsystem),
        //                 new InstantCommand(() -> swerveSubsystem.stopModules()));

        // A chooser for autonomous commands
        SendableChooser<Command> m_chooser = new SendableChooser<>();

        public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        public final static ArmSubsystem armSubsystem = new ArmSubsystem();
        public final static RotateSubsystem rotateSubsystem = new RotateSubsystem();

        public final XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);
        public final static XboxController secondaryJoystick = new XboxController(
                        OIConstants.kSecondaryDriverControllerPort);

        public RobotContainer() {

                // m_chooser.setDefaultOption("Quick Play Dead", autoPlace);
                // m_chooser.addOption("Who Needs The Community", autoSide);
                // m_chooser.addOption("SEE-SAW", autoMiddle);
                // SmartDashboard.putData("Auto choices", m_chooser);

                // Put the chooser on the dashboard
                Shuffleboard.getTab("Autonomous").add(m_chooser);

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> driverJoystick.getRawButton(OIConstants.kDriverlbumper),
                                () -> driverJoystick.getRawButton(OIConstants.kDriverrbumper),
                                () -> driverJoystick.getRawButton(OIConstants.kDriverSlowButton)));

                configureButtonBindings();
        }

        private void configureButtonBindings() {
                // Primary

                new JoystickButton(driverJoystick, 3)
                                .whileTrue(new DriveAllignPoleCmd(swerveSubsystem));

                new JoystickButton(driverJoystick, 4).onTrue(new DriveGyroResetCmd(swerveSubsystem));

                new JoystickButton(driverJoystick, 2).whileTrue(new AutoChargingBalanceCmd(swerveSubsystem));

                new JoystickButton(driverJoystick, ).onTrue(Commands.parallel(new ArmManipulatorDriveCmd(rotateSubsystem)));

                new JoystickButton(driverJoystick, ).onTrue(Commands.parallel(new ArmManipulatorIntakeCmd(rotateSubsystem)));

                new JoystickButton(driverJoystick, ).whileTrue(new ArmIntakeInCmd(armSubsystem));

                new JoystickButton(driverJoystick, ).whileTrue(new ArmIntakeOutCmd(armSubsystem));
        }

        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
        }
}