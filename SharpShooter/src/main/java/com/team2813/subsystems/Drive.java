package com.team2813.subsystems;

import static com.team2813.Constants.BACK_LEFT_DRIVE_ID;
import static com.team2813.Constants.BACK_LEFT_ENCODER_ID;
import static com.team2813.Constants.BACK_LEFT_STEER_ID;
import static com.team2813.Constants.BACK_RIGHT_DRIVE_ID;
import static com.team2813.Constants.BACK_RIGHT_ENCODER_ID;
import static com.team2813.Constants.BACK_RIGHT_STEER_ID;
import static com.team2813.Constants.FRONT_LEFT_DRIVE_ID;
import static com.team2813.Constants.FRONT_LEFT_ENCODER_ID;
import static com.team2813.Constants.FRONT_LEFT_STEER_ID;
import static com.team2813.Constants.FRONT_RIGHT_DRIVE_ID;
import static com.team2813.Constants.FRONT_RIGHT_ENCODER_ID;
import static com.team2813.Constants.FRONT_RIGHT_STEER_ID;
import static com.team2813.Constants.PIGEON_ID;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {

    public static final double MAX_VELOCITY = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI; // m/s
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 8; // radians per second

    private double multiplier = 1;

	SwerveDrivetrain drivetrain;

    public Drive() {

        double FLSteerOffset = 0;
        double FRSteerOffset = 0;
        double BLSteerOffset = 0;
        double BRSteerOffset = 0;

		Slot0Configs steerGains = new Slot0Configs()
			.withKP(50).withKI(0).withKD(0.2)
			.withKS(0).withKV(1.5).withKA(0);

		Slot0Configs driveGains = new Slot0Configs()
			.withKP(2.5).withKI(0).withKD(0)
			.withKS(0).withKV(0).withKA(0);
			
		SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
			.withPigeon2Id(PIGEON_ID)
			.withCANbusName("swerve");
		SwerveModuleConstantsFactory constantCreator = new SwerveModuleConstantsFactory()
			.withDriveMotorGearRatio(6.75)
			.withSteerMotorGearRatio(150.0 / 7)
			.withWheelRadius(1.75)
			.withSlipCurrent(90)
			.withSteerMotorGains(steerGains)
			.withDriveMotorGains(driveGains)
			.withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
			.withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
			.withSpeedAt12VoltsMps(MAX_VELOCITY)
			.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
			.withCouplingGearRatio(3.5)
			.withSteerMotorInverted(true);
		double frontDist = 0.280029; // x
		double leftDist = 0.303514; // y
		SwerveModuleConstants frontLeft = constantCreator.createModuleConstants(
			FRONT_LEFT_STEER_ID, FRONT_LEFT_DRIVE_ID,
			FRONT_LEFT_ENCODER_ID, FLSteerOffset,
			frontDist, leftDist,
			true
		);
		SwerveModuleConstants frontRight = constantCreator.createModuleConstants(
			FRONT_RIGHT_STEER_ID, FRONT_RIGHT_DRIVE_ID,
			FRONT_RIGHT_ENCODER_ID, FRSteerOffset,
			frontDist, -leftDist,
			true
		);
		SwerveModuleConstants backLeft = constantCreator.createModuleConstants(
			BACK_LEFT_STEER_ID, BACK_LEFT_DRIVE_ID,
			BACK_LEFT_ENCODER_ID, BLSteerOffset,
			-frontDist, leftDist,
			true
		);
		SwerveModuleConstants backRight = constantCreator.createModuleConstants(
			BACK_RIGHT_STEER_ID, BACK_RIGHT_DRIVE_ID,
			BACK_RIGHT_ENCODER_ID, BRSteerOffset,
			-frontDist, -leftDist,
			true
		);
		SwerveModuleConstants[] constants = new SwerveModuleConstants[]{frontLeft, frontRight, backLeft, backRight};
		SwerveDrivetrain drivetrain = new SwerveDrivetrain(drivetrainConstants, constants);
		this.drivetrain = drivetrain;
	}

	public void stop() {
		drivetrain.setControl(new SwerveRequest.Idle());
	}


    public void enableSlowMode(boolean enable) {
        multiplier = enable ? 0.4 : 1;
    }

	private final SwerveRequest.FieldCentric xyrRequest =
		new SwerveRequest.FieldCentric()
				.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
				.withSteerRequestType(SteerRequestType.MotionMagicExpo);

	public void drive(double x, double y, double rotation) {
		drivetrain.setControl(
			xyrRequest.withVelocityX(x * multiplier)
				.withVelocityY(y * multiplier)
				.withRotationalRate(rotation)
		);
		
	}
}