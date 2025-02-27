// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.LedSubsystem;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    public RobotState RobState;

    public final CommandXboxController driverXbox = new CommandXboxController(0);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    private final ArmSubsystem arm = new ArmSubsystem();
    private final ElevSubsystem elev = new ElevSubsystem();
    //private final ElevSubsystem led = new LedSubsystem();
    
    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> (driverXbox.getRightX()))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.1)
            .scaleRotation(0.1)
            .allianceRelativeControl(true);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        //drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

        /* 
        driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        driverXbox.y().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
        driverXbox.back().whileTrue(Commands.none());
        driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        driverXbox.rightBumper().onTrue(Commands.none());
        */
        driverXbox.a()
            .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverXbox.b()
            .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driverXbox.x()
            .whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverXbox.y()
            .whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureStateBindings(){

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand("New Auto");
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
