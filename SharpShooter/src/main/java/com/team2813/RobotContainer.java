// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813;

import static com.team2813.Constants.OperatorConstants.INTAKE_ONLY_OUTTAKE;
import static com.team2813.Constants.OperatorConstants.MANUAL_INTAKE;
import static com.team2813.Constants.OperatorConstants.MANUAL_OUTTAKE;
import static com.team2813.Constants.OperatorConstants.OPERATOR_CONTROLLER;

import com.team2813.commands.ElevatorDefaultCommand;
import com.team2813.subsystems.Amp;
import com.team2813.subsystems.Elevator;
import com.team2813.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RobotContainer {
  private final Amp amp = new Amp();
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();

  public RobotContainer() {
	elevator.setDefaultCommand(new ElevatorDefaultCommand(elevator, () -> -OPERATOR_CONTROLLER.getRightY()));
    configureBindings();
  }

  private void configureBindings() {
    MANUAL_INTAKE.onTrue(
      new ParallelCommandGroup(
        new InstantCommand(amp::pushIn, amp),
        new InstantCommand(intake::intake, intake)
      )
    );
    MANUAL_INTAKE.onFalse(
      new ParallelCommandGroup(
        new InstantCommand(amp::stop, amp),
        new InstantCommand(intake::stop, intake)
      )
    );

    MANUAL_OUTTAKE.onTrue(
      new ParallelCommandGroup(
        new InstantCommand(amp::pushOut, amp),
        new InstantCommand(intake::outtake, intake)
      )
    );
    MANUAL_OUTTAKE.onFalse(
      new ParallelCommandGroup(
        new InstantCommand(amp::stop, amp),
        new InstantCommand(intake::stop, intake)
      )
    );

	INTAKE_ONLY_OUTTAKE.onTrue(
		new InstantCommand(intake::outtake, intake)
	);

	INTAKE_ONLY_OUTTAKE.onFalse(
		new InstantCommand(intake::stop, intake)
	);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
