package com.team2813.subsystems;

import static com.team2813.Constants.ELEVATOR_1;
import static com.team2813.Constants.ELEVATOR_2;
import static edu.wpi.first.units.Units.Rotations;

import com.team2813.lib2813.control.ControlMode;
import com.team2813.lib2813.control.InvertType;
import com.team2813.lib2813.control.motors.TalonFXWrapper;
import com.team2813.lib2813.subsystems.MotorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

import java.util.function.Supplier;

public class Elevator extends MotorSubsystem<Elevator.Position> {

  public Elevator() {
    super(
        new MotorSubsystemConfiguration(
                new TalonFXWrapper(ELEVATOR_1, "swerve", InvertType.COUNTER_CLOCKWISE))
            .controlMode(ControlMode.VOLTAGE)
            .PID(1383, 0, 0));
    ((TalonFXWrapper) motor).addFollower(ELEVATOR_2, "swerve", InvertType.OPPOSE_MASTER);
  }
  
  @Override
  protected void useOutput(double output, double setpoint) {
    super.useOutput(MathUtil.clamp(output, -0.5, 0.5), setpoint);
  }
  
  public enum Position implements Supplier<Measure<Angle>> {
    BOTTOM(0.216797),
    TOP(19.709961);

    private final Measure<Angle> position;

    Position(double position) {
      this.position = Rotations.of(position);
    }

    @Override
    public Measure<Angle> get() {
      return position;
    }
  }
}
