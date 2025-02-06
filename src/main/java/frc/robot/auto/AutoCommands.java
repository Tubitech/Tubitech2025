package frc.robot.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.RobotContainer;
import frc.robot.commands.GetCoral;
import frc.robot.commands.PutCoral;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class AutoCommands {
    private RobotContainer robotContainer;
    private final AutoFactory autoFactory;
    
    private final AutoChooser autoChooser;
    public AutoCommands(RobotContainer robotContainer){
        this.robotContainer = robotContainer;

        this.autoFactory = new AutoFactory(robotContainer.getDrivebase()::getPose, robotContainer.getDrivebase()::resetOdometry, robotContainer.getDrivebase()::followTrajectory, true, robotContainer.getDrivebase());
    
        autoChooser = new AutoChooser();
        // Add options to the chooser
        autoChooser.addRoutine("Example Routine",this::ScoreCoral);
        // Put the auto chooser on the dashboard
        SmartDashboard.putData(autoChooser);
        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }
    public AutoRoutine ScoreCoral(){
        AutoRoutine routine = autoFactory.newRoutine("pickUpAndScoreCoral");
        AutoTrajectory driveToReef = routine.trajectory("startToReef");
        AutoTrajectory reefToSource = routine.trajectory("reefToSource");
        AutoTrajectory sourceToReef = routine.trajectory("sourceToReef");

        routine.active().onTrue(
            Commands.sequence(
                driveToReef.resetOdometry(),
                driveToReef.cmd()
            )
        );
        driveToReef.atTime("startScoring").onTrue(robotContainer.putCoralCommands.L1());
        driveToReef.done().onTrue(reefToSource.cmd());

        reefToSource.done().onTrue(robotContainer.getCoralCommands.FromSourceVertical().andThen(sourceToReef.cmd()));
        sourceToReef.atTime("startScoring2").onTrue(robotContainer.putCoralCommands.L1());
        return routine;
    }
}
