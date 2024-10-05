package com.team2813.commands;

import com.team2813.subsystems.Drive;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class DefaultDriveCommand extends Command {
  private final Drive drive;
  private final Supplier<Measure<Velocity<Distance>>> xSupplier;
  private final Supplier<Measure<Velocity<Distance>>> ySupplier;
  private final Supplier<Measure<Velocity<Angle>>> rotationSupplier;
  public DefaultDriveCommand(
          Drive drive,
          Supplier<Measure<Velocity<Distance>>> xSupplier,
          Supplier<Measure<Velocity<Distance>>> ySupplier,
          Supplier<Measure<Velocity<Angle>>> rotationSupplier
  ) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;
    addRequirements(drive);
  }
  
  @Override
  public void execute() {
    drive.drive(
        xSupplier.get().in(Units.MetersPerSecond),
        ySupplier.get().in(Units.MetersPerSecond),
        rotationSupplier.get().in(Units.RadiansPerSecond)
    );
  }
}
