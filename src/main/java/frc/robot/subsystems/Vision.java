package frc.robot.subsystems;


import static org.wpilib.units.Units.Meters;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.SubsystemBase;

import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateConfig;
import com.limelightvision.PoseEstimateType;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.utils.Field;

@SuppressWarnings("resource")

public class Vision extends SubsystemBase {

    // DON'T FORGET TO UPDATE THE NAME OF THE CAMERAS IN THE CONSTRUCTOR BELOW.

    private static final String FRONT_LIMELIGHT_NAME = "limelight-front";
    private static final String LEFT_LIMELIGHT_NAME = "limelight-left";
    private static final String RIGHT_LIMELIGHT_NAME = "limelight-right";

    /**
     * MegaTag2 filtering + std devs. The library rejects bad estimates and
     * fills in est.stdDevs for us. Starts from Limelight's (untuned) defaults:
     * XY std dev = 0.3 * avgTagDistance / sqrt(tagCount), heading not fused.
     */
    private static PoseEstimateConfig mt2Config() {
        return PoseEstimateConfig.defaultMT2()
            .withFieldBounds(Field.fieldLength.in(Meters), Field.fieldWidth.in(Meters))
            .withStdDevXY(0.3, 0.05, 5.0); // base, min, max (meters) - tune this
    }

    //GET THE COORDINATES FOR THE CAMERAS, X, Y, Z, PITCH, ROLL, YAW
    // Constructor order is: name, forward, left, up (meters), roll, pitch, yaw (degrees)

    private final Limelight frontLimelight = new Limelight(
        FRONT_LIMELIGHT_NAME,
        VisionConstants.FRONT_LIMELIGHT_X, VisionConstants.FRONT_LIMELIGHT_Y, VisionConstants.FRONT_LIMELIGHT_Z,
        VisionConstants.FRONT_LIMELIGHT_ROLL, VisionConstants.FRONT_LIMELIGHT_PITCH, VisionConstants.FRONT_LIMELIGHT_YAW
    ).withPoseEstimateConfig_MT2(mt2Config());
 
    private final Limelight leftLimelight = new Limelight(
        LEFT_LIMELIGHT_NAME,
        VisionConstants.LEFT_LIMELIGHT_X, VisionConstants.LEFT_LIMELIGHT_Y, VisionConstants.LEFT_LIMELIGHT_Z,
        VisionConstants.LEFT_LIMELIGHT_ROLL, VisionConstants.LEFT_LIMELIGHT_PITCH, VisionConstants.LEFT_LIMELIGHT_YAW
    ).withPoseEstimateConfig_MT2(mt2Config());
 
    private final Limelight rightLimelight = new Limelight(
        RIGHT_LIMELIGHT_NAME,
        VisionConstants.RIGHT_LIMELIGHT_X, VisionConstants.RIGHT_LIMELIGHT_Y, VisionConstants.RIGHT_LIMELIGHT_Z,
        VisionConstants.RIGHT_LIMELIGHT_ROLL, VisionConstants.RIGHT_LIMELIGHT_PITCH, VisionConstants.RIGHT_LIMELIGHT_YAW
    ).withPoseEstimateConfig_MT2(mt2Config());

    private final Limelight[] limelights = {frontLimelight, leftLimelight, rightLimelight};

    private static boolean isBlue = false;
    private static boolean isRed = false;

    private final CommandSwerveDrivetrain swerve =
        CommandSwerveDrivetrain.getInstance();

    public Vision() {
        // Apply the MegaTag2 filtering / std dev config to every camera.
        for (Limelight ll : limelights) {
            ll.withPoseEstimateConfig_MT2(mt2Config());
        }
    }

    /**
     * Returns whether a robot pose is outside the playable field dimensions.
     */
    public boolean isEstOffField(Pose3d estimatedPose) {
        if (estimatedPose.getX() < 0
            || estimatedPose.getX() > Field.fieldLength.in(Meters)) {
            return true;
        }

        if (estimatedPose.getY() < 0
            || estimatedPose.getY() > Field.fieldWidth.in(Meters)) {
            return true;
        }

        return false;
    }

    public void resetPosetoTower() {
        if (MatchState.getAlliance().get() == Alliance.BLUE) {
            swerve.resetPose(
                new Pose2d(Field.BlueTower.center, Rotation2d.ZERO)
            );
        }
    }

    public double[] getHubOffsetForTag(int id) {
        switch (id) {
            case 21: return new double[] {0.0, TurretConstants.tagOffset};
            case 26: return new double[] {-TurretConstants.tagOffset, 0.0};
            case 18: return new double[] {0.0, -TurretConstants.tagOffset};

            case 2: return new double[] {0.0, TurretConstants.tagOffset};
            case 10: return new double[] {TurretConstants.tagOffset, 0.0};
            case 5: return new double[] {0.0, -TurretConstants.tagOffset};

            default: return new double[] {0.0, 0.0};
        }
    }

    public Pose3d getHub3D() {
        if (!isBlue && !isRed) {
            if (RobotState.isDSAttached()) {
                isBlue = MatchState.getAlliance().get() == Alliance.BLUE;
                isRed = MatchState.getAlliance().get() == Alliance.RED;
            } else {
                isBlue = false;
                isRed = false;
            }
        }

        Pose3d blueHub = VisionConstants.blueHub;
        Pose3d redHub = VisionConstants.redHub;

        if (isRed) {
            return redHub;
        } else {
            return blueHub;
        }
    }

    public Rotation2d getTrenchAngle(double x) {
        if (Field.isBlue()) {
            if (x > 4.63) {
                return new Rotation2d(Math.toRadians(0));
            } else {
                return new Rotation2d(Math.toRadians(180));
            }
        } else {
            if (x > 11.91) {
                return new Rotation2d(Math.toRadians(180));
            } else {
                return new Rotation2d(Math.toRadians(0));
            }
        }
    }

    /**
     * Returns whether an AprilTag belongs to the hub on the current alliance.
     */
    public boolean isTagHub(int tagId) {
        if (Field.isBlue()) {
            return tagId == 18 || tagId == 19 || tagId == 20 || tagId == 21
                || tagId == 24 || tagId == 25 || tagId == 26 || tagId == 27;
        }

        if (Field.isRed()) {
            return tagId == 8 || tagId == 9 || tagId == 10 || tagId == 11
                || tagId == 2 || tagId == 3 || tagId == 4 || tagId == 5;
        }

        return false;
    }

    @Override
    public void periodic() {
        var state = swerve.getState();
        double yawDeg = state.Pose.getRotation().getDegrees();
        double yawRateDegPerSec = Math.toDegrees(state.Velocity.omega);

        // 1) Publish heading BEFORE reading MegaTag2. One call updates every camera.
        //    If you already call this elsewhere, delete this line and make sure
        //    yours runs every loop before Vision.periodic().
        Limelight.setSharedRobotOrientation(yawDeg, yawRateDegPerSec, 0, 0, 0, 0);

        for (Limelight ll : limelights) {
            // 2) Every estimate since last loop that passed the MT2 config filters.
            //    (This consumes the queue - only read it once per camera per loop.)
            PoseEstimate[] estimates = ll.readAcceptedPoseEstimates(PoseEstimateType.MT2_WPIBLUE);

            // 3) Feed each one to the swerve pose estimator.
            for (PoseEstimate est : estimates) {
                swerve.addVisionMeasurement(est.pose, est.timestampSeconds, est.stdDevs);
            }
        }
    }
}