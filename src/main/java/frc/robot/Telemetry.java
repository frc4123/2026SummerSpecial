package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.smartdashboard.Field2d;
// import frc.robot.subsystems.CommandSwerveDrivetrain;


import org.littletonrobotics.junction.Logger;

public class Telemetry {

    private final Field2d m_field = new Field2d();

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable driveStateTable = inst.getTable("DriveState");

    private final StructPublisher<Pose2d> drivePose =
        driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

    public Telemetry(double maxSpeed) {
        org.wpilib.telemetry.Telemetry.log("Robot Field", m_field);
    }

    public void telemeterize(SwerveDriveState state) {
        Pose2d pose = state.Pose;
        drivePose.set(pose);
        m_field.setRobotPose(pose);
        Logger.recordOutput("Drive/Pose", pose);
        org.wpilib.telemetry.Telemetry.log("Debug/PoseX", pose.getX());
        org.wpilib.telemetry.Telemetry.log("Debug/PoseY", pose.getY());
        org.wpilib.telemetry.Telemetry.log("Debug/PoseDeg", pose.getRotation().getDegrees());
    }
}
