package com.team2813.subsystems;

import static com.team2813.Constants.AMP;

import com.team2813.lib2813.control.ControlMode;
import com.team2813.lib2813.control.InvertType;
import com.team2813.lib2813.control.Motor;
import com.team2813.lib2813.control.motors.TalonFXWrapper;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amp extends SubsystemBase {
  private final Motor gary;

  public Amp() {
    gary = new TalonFXWrapper(AMP, "swerve", InvertType.CLOCKWISE);
  }

  public void pushIn() {
    gary.set(ControlMode.VOLTAGE, 10);
  }
  
  public void autoPushIn() {
    gary.set(ControlMode.VOLTAGE, 12);
  }

  public void pushOut() {
    gary.set(ControlMode.VOLTAGE, -6);
  }

  public void shootAmp() {
    gary.set(ControlMode.VOLTAGE, -10);
  }

  public void stop() {
    gary.set(ControlMode.VOLTAGE, 0);
  }
}
