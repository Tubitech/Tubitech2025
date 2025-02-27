// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StateChanger extends SubsystemBase {
  RobotState state;
  ArmSubsystem arm;
  ElevSubsystem elev;
  GripperSubsystem gripper;
  ClimbSubsystem climb;
  LedSubsystem led;

  public StateChanger(RobotState s, ArmSubsystem _arm, ElevSubsystem _elev, GripperSubsystem _gripper, ClimbSubsystem _climb, LedSubsystem _led) {
    state = s;
    arm = _arm;
    elev = _elev;
    gripper = _gripper;
    climb = _climb;
    led = _led;
  }

  public Command changeAlkoy(boolean koy) {
    return runOnce(() -> {
      if(koy) state.alkoy = RobotState.State1.koy;
      else state.alkoy = RobotState.State1.al;
      led.update(state);
    });
  }

  public Command changeAlgae(boolean algae) {
    return runOnce(() -> {
      if(algae) state.coralg = RobotState.State2.alg;
      else state.coralg = RobotState.State2.coral;
      led.update(state);
    }); 
  }

  public Command changeLR(boolean left) {
    return runOnce(() -> {
      if(left) state.yon = RobotState.State3.sol;
      else state.yon = RobotState.State3.sag;
      led.update(state);
    });
  }

  public int getState() {
    return state.getint();
  }

  public Command execute(int btn) { // A, B, X, Y
    // TODO
    return Commands.none();
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}