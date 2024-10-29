package com.team2813;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Constants {
  // Front Right swerve module
  public static final int FRONT_RIGHT_STEER_ID = 1;
  public static final int FRONT_RIGHT_ENCODER_ID = 2;
  public static final int FRONT_RIGHT_DRIVE_ID = 3;
  // Back Right swerve module
  public static final int BACK_RIGHT_STEER_ID = 4;
  public static final int BACK_RIGHT_ENCODER_ID = 5;
  public static final int BACK_RIGHT_DRIVE_ID = 6;
  // Back Left swerve module
  public static final int BACK_LEFT_STEER_ID = 7;
  public static final int BACK_LEFT_ENCODER_ID = 8;
  public static final int BACK_LEFT_DRIVE_ID = 9;
  // Front Left swerve module
  public static final int FRONT_LEFT_STEER_ID = 10;
  public static final int FRONT_LEFT_ENCODER_ID = 11;
  public static final int FRONT_LEFT_DRIVE_ID = 12;
  public static final int PIGEON_ID = 13;
  // roboRIO can loop
  public static final int ELEVATOR_1 = 14;
  public static final int ELEVATOR_2 = 15;
  public static final int AMP = 16;
  public static final int INTAKE = 17;

  private Constants() {
    throw new AssertionError("Not instantiable");
  }

  public static class DriverConstants {
    public static CommandPS4Controller DRIVER_CONTROLLER = new CommandPS4Controller(0);
    public static Trigger SLOW_MODE = DRIVER_CONTROLLER.L1();
    public static Trigger MANUAL_OUTTAKE = DRIVER_CONTROLLER.R2();
    public static Trigger MANUAL_INTAKE = DRIVER_CONTROLLER.R1();
    public static Trigger TEST = DRIVER_CONTROLLER.cross();

    private DriverConstants() {
      throw new AssertionError("Not instantiable");
    }
  }

  public static class OperatorConstants {
    public static CommandPS4Controller OPERATOR_CONTROLLER = new CommandPS4Controller(1);
    public static Trigger INTAKE = OPERATOR_CONTROLLER.R1();
    public static Trigger OUTTAKE = OPERATOR_CONTROLLER.L1();
    
    public static Trigger AMP = OPERATOR_CONTROLLER.R2();
    public static Trigger ZERO_ELEVATOR_TOP = OPERATOR_CONTROLLER.share();
    public static Trigger ZERO_ELEVATOR_BOTTOM = OPERATOR_CONTROLLER.options();
    public static Trigger INTAKE_ONLY_OUTTAKE = OPERATOR_CONTROLLER.povUp();

    private OperatorConstants() {
      throw new AssertionError("Not instantiable");
    }
  }
}
