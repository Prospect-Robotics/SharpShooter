package com.team2813;

public class Constants {
	private Constants() {
		throw new AssertionError("Not intantionable");
	}

	public static class DriverConstants {
		private DriverConstants() {
			throw new AssertionError("Not intantionable");
		}
	}

	public static class OperatorConstants {
		private OperatorConstants() {
			throw new AssertionError("Not intantionable");
		}
	}

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
}
