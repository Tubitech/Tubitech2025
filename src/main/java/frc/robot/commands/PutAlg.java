package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class PutAlg {
    private RobotContainer robotContainer;
    public PutAlg(RobotContainer robotContainer){
        this.robotContainer = robotContainer;
    }
    public Command ToBarge(){
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pushAlg());
    }
    public Command ToProcessor(){
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pushAlg());
    }
}
