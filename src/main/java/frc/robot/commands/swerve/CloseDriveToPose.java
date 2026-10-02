package frc.robot.commands.swerve;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.command2.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class CloseDriveToPose extends Command {

    private final CommandSwerveDrivetrain swerve;
    private Pose2d poseFinal, currentPose;

    private final PIDController xTranslationPID, yTranslationPID;
    private final PIDController rotationPID;

    public CloseDriveToPose(CommandSwerveDrivetrain swerve, Pose2d finalPose) {
        this.swerve = swerve;
        this.poseFinal = finalPose;

        this.xTranslationPID = new PIDController(Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KP, 
            Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KI, 
            Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KD);
        this.yTranslationPID = new PIDController(Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KP, 
            Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KI, 
            Constants.SwerveConstants.CLOSE_TRANSLATION_PP_KD);
        this.rotationPID = new PIDController(Constants.SwerveConstants.CLOSE_ROTATION_PP_KP, 
            Constants.SwerveConstants.CLOSE_ROTATION_PP_KI, 
            Constants.SwerveConstants.CLOSE_ROTATION_PP_KD);
        
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);

        xTranslationPID.setTolerance(0.002);
        yTranslationPID.setTolerance(0.002);
        rotationPID.setTolerance(0.01);
        
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        currentPose = swerve.getState().Pose;

        xTranslationPID.reset();
        yTranslationPID.reset();
        rotationPID.reset();

        xTranslationPID.setSetpoint(poseFinal.getX());
        yTranslationPID.setSetpoint(poseFinal.getY());
        rotationPID.setSetpoint(poseFinal.getRotation().getRadians());
    }

    @Override
    public void execute() {
        currentPose = swerve.getState().Pose;
        
        double xSpeed = xTranslationPID.calculate(currentPose.getX());
        double ySpeed = yTranslationPID.calculate(currentPose.getY());
        double thetaSpeed = rotationPID.calculate(currentPose.getRotation().getRadians());
        
        ChassisVelocities wheelSpeeds = new ChassisVelocities(xSpeed, ySpeed, thetaSpeed);

        swerve.setControl(new SwerveRequest.ApplyFieldVelocity().withVelocity(wheelSpeeds));
    }

    @Override
    public boolean isFinished() {
        boolean xTranslationDone = xTranslationPID.atSetpoint();
        boolean yTranslationDone = yTranslationPID.atSetpoint();
        boolean rotationDone = rotationPID.atSetpoint();
        
        return xTranslationDone && yTranslationDone && rotationDone;
    }
    
    @Override
    public void end(boolean interrupted) {
        swerve.setControl(new SwerveRequest.ApplyRobotVelocity().withVelocity(new ChassisVelocities(0, 0, 0)));
    }
}