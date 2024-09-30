package com.team2813.commands;

import com.team2813.lib2813.control.ControlMode;
import com.team2813.subsystems.Elevator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Objects;

import com.team2813.subsystems.Elevator.Position;

public class ZeroElevatorCommand extends Command {
  private final Elevator elevator;
  private final Position position;
  private double startTime = 0;
  
  /**
   *
   * @param elevator the elevator
   * @throws NullPointerException when elevator is null
   */
  public ZeroElevatorCommand(Elevator elevator, Position position) {
    this.elevator = Objects.requireNonNull(elevator);
    if (position != Position.BOTTOM && position != Position.TOP) {
      throw new IllegalArgumentException(String.format("%s is not a supported position", position.toString()));
    }
    this.position = position;
    addRequirements(elevator);
  }
  
  @Override
  public void initialize() {
    double speed = switch (position) {
      case TOP -> 0.75;
      case BOTTOM -> -0.70;
      default -> throw new AssertionError("Should be impossible");
    };
    elevator.set(ControlMode.DUTY_CYCLE, speed);
    startTime = Timer.getFPGATimestamp();
  }
  
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime > 0.25 && Math.abs(elevator.getVelocityMeasure().in(Units.DegreesPerSecond)) < 5;
  }
  
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      elevator.setPosition(position.get());
    }
    elevator.set(ControlMode.DUTY_CYCLE, 0);
  }
}