package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class GetCoral {
    private RobotContainer robotContainer;
    public GetCoral(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
    }
    public Command FromSource(){
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pullCoral());
    }
    public Command FromGroundVertical(){
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pullCoral());
    }
    public Command FromGroundHorizontal(){
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pullCoral());
    }
}
