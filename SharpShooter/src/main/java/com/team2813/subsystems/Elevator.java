package com.team2813.subsystems;

import com.team2813.lib2813.control.ControlMode;
import com.team2813.lib2813.control.InvertType;
import com.team2813.lib2813.control.motors.TalonFXWrapper;
import com.team2813.lib2813.subsystems.MotorSubsystem;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

import java.util.function.Supplier;

import static com.team2813.Constants.ELEVATOR_1;
import static com.team2813.Constants.ELEVATOR_2;
import static edu.wpi.first.units.Units.Degrees;

public class Elevator extends MotorSubsystem<Elevator.Position> {
    
     public Elevator() {
        super(
            new MotorSubsystemConfiguration(new TalonFXWrapper(ELEVATOR_1,InvertType.CLOCKWISE))
                    .controlMode(ControlMode.VOLTAGE)
        );
        ((TalonFXWrapper) motor).addFollower(ELEVATOR_2,InvertType.OPPOSE_MASTER);
    }

    public enum Position implements Supplier<Measure<Angle>> {
        TEST(0),
        BOTTOM(0),
        TOP(0);

        private final Measure<Angle> position;
        Position(double position) {
            this.position = Degrees.of(position);
        }
        @Override
        public Measure<Angle> get() {
            return position;
        }
       
    }
}
