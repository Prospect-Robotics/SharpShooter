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
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;


public class Drive extends SubsystemBase {

  public static final double MAX_VELOCITY = 10; // mps
  public static final double MAX_ROTATION = Math.PI * 3.5; // radians per second
  private final SwerveRequest.FieldCentric xyrRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  SwerveDrivetrain drivetrain;
  private double multiplier = 1;
  private static class PublicisizedKinematics extends SwerveDrivetrain {
    public PublicisizedKinematics(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
      super(driveTrainConstants, modules);
    }

    public ChassisSpeeds getChassisSpeeds() {
      return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }
  }
  public Drive() {

    double FLSteerOffset = 0.4908046875;
    double FRSteerOffset = 0.28930664062;
    double BLSteerOffset = -0.03442382812;
    double BRSteerOffset = -0.10009765625;

    Slot0Configs steerGains =
        new Slot0Configs().withKP(50).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);

    Slot0Configs driveGains =
        new Slot0Configs().withKP(2.5).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    SwerveDrivetrainConstants drivetrainConstants =
        new SwerveDrivetrainConstants().withPigeon2Id(PIGEON_ID).withCANbusName("rio");
    SwerveModuleConstantsFactory constantCreator =
        new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(6.75)
            .withSteerMotorGearRatio(150.0 / 7)
            .withWheelRadius(1.75)
            .withSlipCurrent(90)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.TorqueCurrentFOC)
            .withSpeedAt12VoltsMps(5)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(3.5)
            .withSteerMotorInverted(true);
    double frontDist = 0.280029; // x
    double leftDist = 0.303514; // y
    SwerveModuleConstants frontLeft =
        constantCreator.createModuleConstants(
            FRONT_LEFT_STEER_ID,
            FRONT_LEFT_DRIVE_ID,
            FRONT_LEFT_ENCODER_ID,
            FLSteerOffset,
            frontDist,
            leftDist,
            true);
    SwerveModuleConstants frontRight =
        constantCreator.createModuleConstants(
            FRONT_RIGHT_STEER_ID,
            FRONT_RIGHT_DRIVE_ID,
            FRONT_RIGHT_ENCODER_ID,
            FRSteerOffset,
            frontDist,
            -leftDist,
            true);
    SwerveModuleConstants backLeft =
        constantCreator.createModuleConstants(
            BACK_LEFT_STEER_ID,
            BACK_LEFT_DRIVE_ID,
            BACK_LEFT_ENCODER_ID,
            BLSteerOffset,
            -frontDist,
            leftDist,
            true);
    SwerveModuleConstants backRight =
        constantCreator.createModuleConstants(
            BACK_RIGHT_STEER_ID,
            BACK_RIGHT_DRIVE_ID,
            BACK_RIGHT_ENCODER_ID,
            BRSteerOffset,
            -frontDist,
            -leftDist,
            true);
    SwerveModuleConstants[] constants =
        new SwerveModuleConstants[] {frontLeft, frontRight, backLeft, backRight};
    PublicisizedKinematics drivetrain = new PublicisizedKinematics(drivetrainConstants, constants);
    this.drivetrain = drivetrain;
    RobotConfig config = new RobotConfig(49.07, 5.26, new ModuleConfig(0.044, MAX_VELOCITY, 1.0,
            DCMotor.getKrakenX60(1).withReduction(6.75),40.0, 1), 0.85, 0.939);
    AutoBuilder.configure(
            this::getAutoPose,
            this::resetOdometry,
            drivetrain::getChassisSpeeds,
            this::drive,
            new PPHolonomicDriveController(
                    new PIDConstants(0, 0, 0), // Translation PID
                    new PIDConstants(0, 0, 0) // Rotation PID
            ),
            config,
            Drive::onRed,
            this);
    for (int i = 0; i < 4; i++) {
      int temp = i;
      Shuffleboard.getTab("swerve").addDouble(String.format("Module [%d] position", i), () -> getPosition(temp));
    }
  }
  
  private double getPosition(int moduleId) {
    return drivetrain.getModule(moduleId).getCANcoder().getAbsolutePosition().getValueAsDouble();
  }
  private static boolean onRed() {
    return DriverStation.getAlliance()
            .<Boolean>map((j) -> j == DriverStation.Alliance.Red)
            .orElse(false);
  }
  public Pose2d getAutoPose() {
      return drivetrain.getState().Pose;
  }
  public void resetOdometry(Pose2d currentPose) {
    drivetrain.seedFieldRelative(currentPose);
  }

  public void stop() {
    drivetrain.setControl(new SwerveRequest.Idle());
  }
  private final SwerveRequest.ApplyChassisSpeeds chassisSpeedsRequest =
          new SwerveRequest.ApplyChassisSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  public void enableSlowMode(boolean enable) {
    multiplier = enable ? 0.4 : 1;
  }
  public void drive(ChassisSpeeds demand) {
    drivetrain.setControl(chassisSpeedsRequest.withSpeeds(demand));
  }
  public void drive(double x, double y, double rotation) {
    DriverStation.reportWarning(String.format("x: %f, y: %f, Ø: %f", x, y, rotation), false);
    drivetrain.setControl(
        xyrRequest
            .withVelocityX(x * multiplier)
            .withVelocityY(y * multiplier)
            .withRotationalRate(rotation));
  }

  
  public Rotation2d getRotation() {
    return drivetrain.getRotation3d().toRotation2d();
  }
  
  StructArrayPublisher<SwerveModuleState> expectedState =
          NetworkTableInstance.getDefault().getStructArrayTopic("expected state", SwerveModuleState.struct).publish();
  StructArrayPublisher<SwerveModuleState> actualState =
          NetworkTableInstance.getDefault().getStructArrayTopic("actual state", SwerveModuleState.struct).publish();
  StructPublisher<Rotation2d> rotation =
          NetworkTableInstance.getDefault().getStructTopic("rotation", Rotation2d.struct).publish();
  
  @Override
  public void periodic() {
    expectedState.set(drivetrain.getState().ModuleTargets);
    actualState.set(drivetrain.getState().ModuleStates);
    rotation.set(getRotation());
  }
}