package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.swervedrive.auto.AlignToAprilTagCommand;
import frc.robot.subsystems.ArmElevatorSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SetArmCommand extends Command{
    private final double l4X ;
    private final double l4Y ;
    private final double l4H ;

    private GripperSubsystem gripperSubsystem;
    private ArmElevatorSubsystem armElevatorSubsystem;
    private SwerveSubsystem swerveSubsystem;
    public SetArmCommand(SwerveSubsystem swerve ,GripperSubsystem gripper, ArmElevatorSubsystem armElevator, double x, double y, double h){
        gripperSubsystem = gripper;
        armElevatorSubsystem = armElevator;
        swerveSubsystem = swerve;
        l4H = h;
        l4X = x;
        l4Y = y;
        addRequirements(gripperSubsystem, armElevatorSubsystem, swerveSubsystem);
    }
    @Override
    public void execute(){
        armElevatorSubsystem.setTargetX(l4X);
        armElevatorSubsystem.setTargetY(l4Y);
        armElevatorSubsystem.setTargetH(l4H);

    }
    @Override
    public boolean isFinished(){
        return armElevatorSubsystem.isInPosition();
    }
}
