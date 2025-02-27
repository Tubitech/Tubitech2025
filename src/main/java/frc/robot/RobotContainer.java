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
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.StateChanger;

import java.io.File;
import swervelib.SwerveInputStream;

public class RobotContainer {

    public RobotState state;

    public final CommandXboxController driverXbox = new CommandXboxController(0);
    private final SwerveSubsystem drivebase = new SwerveSubsystem(
            new File(Filesystem.getDeployDirectory(), "swerve/neo"));

    private final ArmSubsystem arm = new ArmSubsystem();
    private final ElevSubsystem elev = new ElevSubsystem();
    private final LedSubsystem led = new LedSubsystem();
    private final GripperSubsystem gripper = new GripperSubsystem(state);
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final StateChanger stateChanger = new StateChanger(state, arm, elev, gripper, climb, led);
    
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> (driverXbox.getRightX()))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.1)
            .scaleRotation(0.1)
            .allianceRelativeControl(true);

    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    }

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
        driverXbox.leftTrigger().onTrue(stateChanger.changeAlgae(false));
        driverXbox.rightTrigger().onTrue(stateChanger.changeAlgae(true));

        driverXbox.leftBumper().onTrue(stateChanger.changeAlkoy(false));
        driverXbox.rightBumper().onTrue(stateChanger.changeAlkoy(true));

        driverXbox.povLeft().onTrue(stateChanger.changeLR(true));
        driverXbox.povRight().onTrue(stateChanger.changeLR(false));

        driverXbox.a().onTrue(stateChanger.execute(0));
        driverXbox.b().onTrue(stateChanger.execute(1));
        driverXbox.x().onTrue(stateChanger.execute(2));
        driverXbox.y().onTrue(stateChanger.execute(3));
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand("New Auto");
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
