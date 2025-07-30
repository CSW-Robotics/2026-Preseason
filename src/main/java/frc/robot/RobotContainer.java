// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;

public class RobotContainer {

    private final LimeLight  m_frontLimelight = new LimeLight("limelight");
    private final LimeLight     m_backLimelight = new LimeLight("limelight-front");

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final RobotCentric r_drive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Joystick joystick = new Joystick(0);
    private final Joystick r_joystick = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private boolean isRobotCentric = false; // Toggle for driving mode

    private final Field2d field = new Field2d();

    public RobotContainer() {

        SmartDashboard.putData("Field", field);
        // NetworkTableInstance.getDefault().getTable("Field").getEntry("Field").setValue(field);

        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                isRobotCentric
                    ? r_drive.withVelocityX(joystick.getY() * MaxSpeed) // Robot-relative forward/backward
                              .withVelocityY(joystick.getX() * MaxSpeed) // Robot-relative left/right
                              .withRotationalRate(r_joystick.getX() * -1) // Robot-relative rotation
                    : drive.withVelocityX(joystick.getY() * MaxSpeed) // Field-relative forward/backward
                           .withVelocityY(joystick.getX() * MaxSpeed) // Field-relative left/right
                           .withRotationalRate(r_joystick.getX() * -1) // Field-relative rotation
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        new JoystickButton(joystick, 5).whileTrue(drivetrain.applyRequest(() -> brake));

        // Toggle between FieldCentric and RobotCentric driving modes on button press
        new JoystickButton(joystick, 7).onTrue(drivetrain.runOnce(() -> isRobotCentric = !isRobotCentric));

        // I have no clue what this does ngl
        // new JoystickButton(joystick,3).whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        new JoystickButton(joystick,1).whileTrue(drivetrain.applyRequest(() ->
            
            r_drive.withVelocityX(joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(r_joystick.getX()*-1) // Drive counterclockwise with negative X (left)
        ));

        // new JoystickButton(joystick,6).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );

        // new JoystickButton(joystick, 4).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press

        new JoystickButton(joystick, 2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void periodic() {


        // If any of the limelights see an april tag they set the robots possition to the possition given my the limelight

        // We will need to add ofsets to these because the limelights will not be mounted in the direct center of the robot
        if (m_frontLimelight.tv == 1){
            Pose2d m_pos2d = new Pose2d(
                m_frontLimelight.Pos_data[0],
                m_frontLimelight.Pos_data[1],
                new Rotation2d(m_frontLimelight.Pos_data[5])
            );
            drivetrain.resetPose(m_pos2d);
        }

        // If any of the limelights see an april tag they set the robots possition to the possition given my the limelight
        if (m_backLimelight.tv == 1){
            Pose2d m_pos2d = new Pose2d(
                m_backLimelight.Pos_data[0],
                m_backLimelight.Pos_data[1],
                new Rotation2d(m_backLimelight.Pos_data[5])
            );
            drivetrain.resetPose(m_pos2d);
        }


        // Gonna be completely honest copilot is helping me figure out how to do this and it said to do this so yeah

        // Ensure drivetrain state is not null
        if (drivetrain.getState() != null && drivetrain.getState().Pose != null) {
            field.setRobotPose(drivetrain.getState().Pose);
        } else {
            System.out.println("Warning: Drivetrain state or pose is null!");
        }



        // This updates the objects on the field if we can get it working we will use limelights to update this
        field.getObject("GameObjects").setPoses(List.of(
            new Pose2d(3.0, 2.0, new Rotation2d(0)), // Example positions
            new Pose2d(5.0, 1.0, new Rotation2d(0))
        ));
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
