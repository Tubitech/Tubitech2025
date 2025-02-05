package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.auto.AlignToAprilTagCommand;
import frc.robot.subsystems.ArmElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class GetCoralFromSource extends Command{
    GripperSubsystem gripperSubsystem;
    ArmElevatorSubsystem armElevatorSubsystem;
    public GetCoralFromSource(GripperSubsystem gripper, ArmElevatorSubsystem armElevator){
        gripperSubsystem = gripper;
        armElevatorSubsystem = armElevator;
        addRequirements(gripperSubsystem,armElevatorSubsystem);
    }
    @Override
    public void execute(){
        armElevatorSubsystem.setArmAndElevator(0, 0, 0);
        gripperSubsystem.pullCoral();
    }

    @Override
    public void end(boolean interrupted){
        armElevatorSubsystem.setArmAndElevator(0, 0, 0);
        gripperSubsystem.hold();
    }
}
