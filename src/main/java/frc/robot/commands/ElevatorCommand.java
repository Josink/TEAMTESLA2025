// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Position;
import frc.robot.positionConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {
  private Elevator elevator;
  private Position position;
  
  private double setpoint;
  private double tolerance = positionConstants.elevatorConstants.ELEVATOR_TOLERANCE;

  
  public ElevatorCommand(Elevator elevator, Position position) {
    this.elevator = elevator;
    this.position = position;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (position) {
      case ALGEA_BARGE:
        setpoint = positionConstants.elevatorConstants.ALGAE_BARGE_POSITION;
        break;
      case ALGEA_L2:
        setpoint = positionConstants.elevatorConstants.ALGAE_L2_POSITION;
        break;
      case ALGEA_L3:
        setpoint = positionConstants.elevatorConstants.ALGAE_L3_POSITION;
        break;
      case ALGEA_PROCESSOR:
        setpoint = positionConstants.elevatorConstants.ALGAE_PROCESSOR_POSITION;
        break;
      case CORAL_L1:
        setpoint = positionConstants.elevatorConstants.CORAL_L1_POSITION;
        break;
      case CORAL_L2:
        setpoint = positionConstants.elevatorConstants.CORAL_L2_POSITION;
        break;
      case CORAL_L3:
        setpoint = positionConstants.elevatorConstants.CORAL_L3_POSITION;
        break;
      case CORAL_L4:
        setpoint = positionConstants.elevatorConstants.CORAL_L4_POSITION;
        break;
      case SAFE_ZONE:
        setpoint = positionConstants.elevatorConstants.ELEVATOR_SAFE_TO_MOVE_ZONE;
        break;
      case INTAKE:
        setpoint = positionConstants.elevatorConstants.STOW_POSITION;
        break;
      case HUMAN_PLAYER_STATION:
        setpoint = positionConstants.elevatorConstants.HUMAN_PLAYER_STATION_POSITION;
        break;
      default:
        break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.GoToPos(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error = elevator.getElevatorPosition() - setpoint;
    return Math.abs(error) < tolerance;
  }
}
