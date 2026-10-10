package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.robot.utils.Field;

import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.networktables.StructPublisher;

import org.littletonrobotics.junction.Logger;

public class Telemetry {

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable driveStateTable = inst.getTable("DriveState");

    private final StructPublisher<Pose2d> drivePose =
        driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

    // Field widget in the old Field2d format that Elastic reads
    private final NetworkTable fieldTable = inst.getTable("SmartDashboard/Field");
    private final StringPublisher fieldType = fieldTable.getStringTopic(".type").publish();
    private final DoubleArrayPublisher fieldRobot = fieldTable.getDoubleArrayTopic("Robot").publish();

    public Telemetry(double maxSpeed) {
        fieldType.set("Field2d");
    }

    public void telemeterize(SwerveDriveState state) {
        Pose2d pose = state.Pose;
        drivePose.set(pose);
        fieldRobot.set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
        Logger.recordOutput("Drive/Pose", pose);
        org.wpilib.telemetry.Telemetry.log("Debug/PoseX", pose.getX());
        org.wpilib.telemetry.Telemetry.log("Debug/PoseY", pose.getY());
        org.wpilib.telemetry.Telemetry.log("Debug/PoseDeg", pose.getRotation().getDegrees());
        org.wpilib.telemetry.Telemetry.log("Debug/FieldisBlue", Field.isBlue());
        org.wpilib.telemetry.Telemetry.log("Debug/MatchStateAlliance", 
        MatchState.getAlliance().map(Object::toString).orElse("none"));
        
    }
}