package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class GetAlg {
    private RobotContainer robotContainer;
    public GetAlg(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
    }
    public Command FromGround(){
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pullAlg());
    }
    public Command FromReefL3(){
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pullAlg());
    }
    public Command FromReefL2(){
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pullAlg());
    }
}
