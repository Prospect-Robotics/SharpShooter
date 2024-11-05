// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2813;

import static com.team2813.Constants.DriverConstants.*;
import static com.team2813.Constants.OperatorConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team2813.commands.DefaultDriveCommand;
import com.team2813.commands.ElevatorDefaultCommand;
import com.team2813.commands.LockFunctionCommand;
import com.team2813.commands.ZeroElevatorCommand;
import com.team2813.subsystems.Amp;
import com.team2813.subsystems.Drive;
import com.team2813.subsystems.Elevator;
import com.team2813.subsystems.Intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;

public class RobotContainer {
    private final Amp amp = new Amp();
    private final Intake intake = new Intake();
    private final Elevator elevator = new Elevator();
    private final Drive drive = new Drive();
    SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

    public RobotContainer() {
        elevator.setDefaultCommand(
                new ElevatorDefaultCommand(elevator, () -> -OPERATOR_CONTROLLER.getRightY()));
        drive.setDefaultCommand(
                new DefaultDriveCommand(
                        drive,
                        () ->
                                Units.MetersPerSecond.of(
                                        modifyAxis(DRIVER_CONTROLLER.getLeftX())
                                                * Drive.MAX_VELOCITY),
                        () ->
                                Units.MetersPerSecond.of(
                                        -modifyAxis(DRIVER_CONTROLLER.getLeftY())
                                                * Drive.MAX_VELOCITY),
                        () ->
                                Units.RadiansPerSecond.of(
                                        -modifyAxis(DRIVER_CONTROLLER.getRightX())
                                                * Drive.MAX_ROTATION)));
        
        SmartDashboard.putData(autoChooser);
        configureBindings();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0) {
                return (value - deadband) / (1 - deadband);
            } else {
                return (value + deadband) / (1 - deadband);
            }
        } else {
            return 0;
        }
    }

    private static double modifyAxis(double value) {
        value = deadband(value, 0.1);
        value = Math.copySign(value * value, value);
        return value;
    }

    private void configureBindings() {
        MANUAL_INTAKE.onTrue(
                new ParallelCommandGroup(
                        new InstantCommand(amp::pushIn, amp),
                        new InstantCommand(intake::intake, intake)));
        MANUAL_INTAKE.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(amp::stop, amp),
                        new InstantCommand(intake::stop, intake)));

        MANUAL_OUTTAKE.onTrue(
                new ParallelCommandGroup(
                        new InstantCommand(amp::pushOut, amp),
                        new InstantCommand(intake::outtake, intake)));
        MANUAL_OUTTAKE.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(amp::stop, amp),
                        new InstantCommand(intake::stop, intake)));

        INTAKE_ONLY_OUTTAKE.onTrue(new InstantCommand(intake::outtake, intake));

        INTAKE_ONLY_OUTTAKE.onFalse(new InstantCommand(intake::stop, intake));

        TEST.onTrue(
                new InstantCommand(() -> elevator.setSetpoint(Elevator.Position.TEST), elevator));
        TEST.onFalse(new InstantCommand(elevator::disable, elevator));

        INTAKE.onTrue(
                new SequentialCommandGroup(
                        new LockFunctionCommand(
                                elevator::atPosition,
                                () -> elevator.setSetpoint(Elevator.Position.BOTTOM),
                                elevator),
                        new ParallelCommandGroup(
                                new InstantCommand(amp::pushIn, amp),
                                new InstantCommand(intake::intake, intake))));

        INTAKE.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(elevator::disable, elevator),
                        new InstantCommand(amp::stop, amp),
                        new InstantCommand(intake::stop, intake)));

        OUTTAKE.onTrue(
                new SequentialCommandGroup(
                        new LockFunctionCommand(
                                elevator::atPosition,
                                () -> elevator.setSetpoint(Elevator.Position.BOTTOM),
                                elevator),
                        new ParallelCommandGroup(
                                new InstantCommand(amp::pushOut, amp),
                                new InstantCommand(intake::outtake, intake))));

        OUTTAKE.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(elevator::disable, elevator),
                        new InstantCommand(amp::stop, amp),
                        new InstantCommand(intake::stop, intake)));

        AMP.onTrue(new LockFunctionCommand(
                elevator::atPosition, () -> elevator.setSetpoint(Elevator.Position.TOP), elevator));

        AMP.onFalse(
                new SequentialCommandGroup(
                        //new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(amp::shootAmp, amp),
                                        new WaitCommand(0.5),
                                        new InstantCommand(amp::stop, amp)),
                                /*Commands.none(),
                                elevator::atPosition),*/
                        new InstantCommand(
                                () -> elevator.setSetpoint(Elevator.Position.BOTTOM), elevator)));

        ZERO_ELEVATOR_TOP.whileTrue(new ZeroElevatorCommand(elevator, Elevator.Position.TOP));
        ZERO_ELEVATOR_BOTTOM.whileTrue(new ZeroElevatorCommand(elevator, Elevator.Position.BOTTOM));

        SLOW_MODE.onTrue(new InstantCommand(() -> drive.enableSlowMode(true), drive));
        SLOW_MODE.onFalse(new InstantCommand(() -> drive.enableSlowMode(false), drive));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
