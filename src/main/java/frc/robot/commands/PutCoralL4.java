package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class PutCoralL4 extends Command{
    GripperSubsystem gripperSubsystem;
    ArmElevatorSubsystem armElevatorSubsystem;
    public PutCoralL4(GripperSubsystem gripper, ArmElevatorSubsystem armElevator){
        gripperSubsystem = gripper;
        armElevatorSubsystem = armElevator;
        addRequirements(gripperSubsystem,armElevatorSubsystem);
    }
    @Override
    public void execute(){
        armElevatorSubsystem.setArmAndElevator(0, 0, 0);
        gripperSubsystem.pushCoral();
    }
    @Override
    public void end(boolean interrupted){
        if (interrupted) {
            /* 1.5 seconds exceeded. So now??? */

        }
        armElevatorSubsystem.setArmAndElevator(0, 0, 0);
        gripperSubsystem.stop();
    }
}
