// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import swerve.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxAngularRate = 4 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final boolean CHARACTERIZATION_ENABLED = true;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final FieldCentricWithProperDeadband drive = new FieldCentricWithProperDeadband()
      .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.15).withRotationalDeadband(MaxAngularRate * 0.15)
      .withMaxSpeed(TunerConstants.kSpeedAt12VoltsMps).withMaxAngularSpeed(MaxAngularRate)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12VoltsMps);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * TunerConstants.kSpeedAt12VoltsMps) // Drive
                                                                                                                    // forward
                                                                                                                    // with
            // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * TunerConstants.kSpeedAt12VoltsMps) // Drive left with negative X
                                                                                     // (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
            .withCreepEnabled(joystick.getRightTriggerAxis() > 0.15) // Drive counterclockwise with negative X (left)
        ));

    joystick.a().and(() -> CHARACTERIZATION_ENABLED).whileTrue(drivetrain.characterizeDrive(1.0, 4.0));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

class FieldCentricWithProperDeadband extends FieldCentric {
  public double MaxSpeed = 0;

  public double MaxAngularSpeed = 0;

  public boolean CreepEnable;

  public double CreepProp = 0.25;

  public double CreepRotProp = 0.25;

  private Rotation2d[] lastRotations = new Rotation2d[4];

  @Override
  public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    double x_direction = VelocityX / Math.abs(VelocityX);
    double y_direction = VelocityY / Math.abs(VelocityY);
    double rot_direction = RotationalRate / Math.abs(RotationalRate);

    double toApplyX = x_direction * Math.max(Math.abs(VelocityX) - Deadband, 0) / (MaxSpeed - Deadband) * MaxSpeed;
    double toApplyY = y_direction * Math.max(Math.abs(VelocityY) - Deadband, 0) / (MaxSpeed - Deadband) * MaxSpeed;
    double toApplyOmega = rot_direction * Math.max(Math.abs(RotationalRate) - RotationalDeadband, 0)
        / (MaxAngularSpeed - RotationalDeadband) * MaxAngularSpeed;

    toApplyX *= CreepEnable ? CreepProp : 1;
    toApplyY *= CreepEnable ? CreepProp : 1;
    toApplyOmega *= CreepEnable ? CreepRotProp : 1;

    ChassisSpeeds speeds = ChassisSpeeds
        .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
            parameters.currentPose.getRotation()), parameters.updatePeriod);

    var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

    for (int i = 0; i < modulesToApply.length; ++i) {
      if (Math.abs(states[i].speedMetersPerSecond) < 0.01) {
        if (lastRotations[i] == null) {
          lastRotations[i] = new Rotation2d();
        }
        states[i].angle = lastRotations[i];
      }
      modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      lastRotations[i] = states[i].angle;
    }

    return StatusCode.OK;
  }

  public FieldCentricWithProperDeadband withMaxSpeed(double maxSpeed) {
    this.MaxSpeed = maxSpeed;
    return this;
  }

  public FieldCentricWithProperDeadband withMaxAngularSpeed(double maxAngularSpeed) {
    this.MaxAngularSpeed = maxAngularSpeed;
    return this;
  }

  public FieldCentricWithProperDeadband withCreepEnabled(boolean creep) {
    this.CreepEnable = creep;
    return this;
  }

  /**
   * Sets the velocity in the X direction, in m/s.
   * X is defined as forward according to WPILib convention,
   * so this determines how fast to travel forward.
   *
   * @param velocityX Velocity in the X direction, in m/s
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withVelocityX(double velocityX) {
    this.VelocityX = velocityX;
    return this;
  }

  /**
   * Sets the velocity in the Y direction, in m/s.
   * Y is defined as to the left according to WPILib convention,
   * so this determines how fast to travel to the left.
   *
   * @param velocityY Velocity in the Y direction, in m/s
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withVelocityY(double velocityY) {
    this.VelocityY = velocityY;
    return this;
  }

  /**
   * The angular rate to rotate at, in radians per second.
   * Angular rate is defined as counterclockwise positive,
   * so this determines how fast to turn counterclockwise.
   *
   * @param rotationalRate Angular rate to rotate at, in radians per second
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withRotationalRate(double rotationalRate) {
    this.RotationalRate = rotationalRate;
    return this;
  }

  /**
   * Sets the allowable deadband of the request.
   *
   * @param deadband Allowable deadband of the request
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withDeadband(double deadband) {
    this.Deadband = deadband;
    return this;
  }

  /**
   * Sets the rotational deadband of the request.
   *
   * @param rotationalDeadband Rotational deadband of the request
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withRotationalDeadband(double rotationalDeadband) {
    this.RotationalDeadband = rotationalDeadband;
    return this;
  }

  /**
   * Sets the type of control request to use for the drive motor.
   *
   * @param driveRequestType The type of control request to use for the drive
   *                         motor
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
    this.DriveRequestType = driveRequestType;
    return this;
  }

  /**
   * Sets the type of control request to use for the steer motor.
   *
   * @param steerRequestType The type of control request to use for the steer
   *                         motor
   * @return this request
   */
  @Override
  public FieldCentricWithProperDeadband withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
    this.SteerRequestType = steerRequestType;
    return this;
  }
}
