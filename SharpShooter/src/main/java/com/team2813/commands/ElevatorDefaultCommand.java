package com.team2813.commands;

import java.util.function.DoubleSupplier;

import com.team2813.lib2813.control.ControlMode;
import com.team2813.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorDefaultCommand extends Command {
	private final Elevator elevator;
	private final DoubleSupplier movement;
	public ElevatorDefaultCommand(Elevator elevator, DoubleSupplier movement) {
		this.elevator = elevator;
		this.movement = movement;
		addRequirements(elevator);
	}
	@Override
	public void execute() {
		elevator.set(ControlMode.DUTY_CYCLE, movement.getAsDouble());
	}
}
