package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotState;

public class PutAlg {
    private RobotContainer robotContainer;
    private RobotState robotState;
    public PutAlg(RobotContainer robotContainer, RobotState robotState){
        this.robotContainer = robotContainer;
        this.robotState = robotState;
    }
    public Command ToBarge(){
        if(!robotState.isAlgAvailable()) return Commands.none();
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pushAlg());
    }
    public Command ToProcessor(){
        if(!robotState.isAlgAvailable()) return Commands.none();
        return new SequentialCommandGroup(new ParallelCommandGroup(robotContainer.getElevator().SetElevAndWaitUntilFinishes(1),robotContainer.getArm().setPositionAndWait(1, 1)),robotContainer.getGripper().pushAlg());
    }
}
