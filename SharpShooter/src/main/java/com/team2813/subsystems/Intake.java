package com.team2813.subsystems;

import static com.team2813.Constants.INTAKE;

import com.team2813.lib2813.control.ControlMode;
import com.team2813.lib2813.control.InvertType;
import com.team2813.lib2813.control.Motor;
import com.team2813.lib2813.control.motors.TalonFXWrapper;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final Motor jerald;

  public Intake() {
    jerald = new TalonFXWrapper(INTAKE, "swerve", InvertType.COUNTER_CLOCKWISE);
  }

  public void intake() {
    jerald.set(ControlMode.VOLTAGE, 1);
  }

  public void outtake() {
    jerald.set(ControlMode.VOLTAGE, -1);
  }

  public void stop() {
    jerald.set(ControlMode.VOLTAGE, 0);
  }
}
