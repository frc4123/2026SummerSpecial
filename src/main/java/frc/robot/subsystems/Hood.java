package frc.robot.subsystems;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.turret.TrajectoryCalculator.ShotData;
import frc.robot.utils.ShotCache;

public class Hood extends SubsystemBase{

    private boolean wasPressed = false;

    private final TalonFX hoodMotor = new TalonFX(
        Constants.CanIdSystemCore.Hood,
        Constants.CanIdSystemCore.canbus4
    );
    
    private final CANdi hoodCANdi = IntakeArmConstants.intakeCANdi;

    private StatusSignal<Boolean> s2Signal = hoodCANdi.getS2Closed();

    private final MotionMagicTorqueCurrentFOC motionMagic =
        new MotionMagicTorqueCurrentFOC(
            HoodConstants.stowPosition
        );

    private final MotionMagicTorqueCurrentFOC motionMagicFree =
        new MotionMagicTorqueCurrentFOC(
            HoodConstants.stowPosition
        );

    public Hood() {
        configureMotor();
    }
   
    private void configureMotor() {

        hoodMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
        
        Slot0Configs pid = new Slot0Configs()
            .withKP(HoodConstants.kP)
            .withKI(HoodConstants.kI)
            .withKD(HoodConstants.kD)
            .withKS(HoodConstants.kS)
            .withKV(HoodConstants.kV)
            .withKA(HoodConstants.kA);

        SoftwareLimitSwitchConfigs softLimits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold((HoodConstants.MAX_HOOD_ANGLE.in(Degrees)))  // rotations
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(HoodConstants.MIN_HOOD_ANGLE.in(Degrees));  // rotations

        FeedbackConfigs feedbackUnits = new FeedbackConfigs()
            .withSensorToMechanismRatio(HoodConstants.sensorToMechanismRatio / 360);

        TorqueCurrentConfigs torqueDeadband = new TorqueCurrentConfigs()
            .withTorqueNeutralDeadband(1.2);

        hoodMotor.getConfigurator().apply(pid);
        hoodMotor.getConfigurator().apply(softLimits);
        hoodMotor.getConfigurator().apply(feedbackUnits);
        hoodMotor.getConfigurator().apply(torqueDeadband);

        zeroHood();   
    }
    
    public void setHoodAngle() {

        ShotData shot = ShotCache.get();

        double desiredAngle = shot.getHoodAngle().in(Degrees);
        desiredAngle *= 0.99;

        hoodMotor.setControl(motionMagic.withPosition(desiredAngle));
    }

    public void lowerHood() {
        hoodMotor.setControl(motionMagic.withPosition(HoodConstants.MAX_HOOD_ANGLE.in(Degrees)));
    }

    public void lowerHoodFree() {
        hoodMotor.setControl(motionMagicFree.withPosition(HoodConstants.ZERO_HOOD_ANGLE.in(Degrees)));

    }

    // public void manualReset() {
    //     SmartDashboard.put
    //     hoodMotor.setPosition(HoodConstants.MAX_HOOD_ANGLE.in(Degrees));
    // }

    // public double getHoodDegrees(){
    //     return hoodPosition.getValueAsDouble();
    // }

    public void zeroHood(){
        hoodMotor.setPosition(HoodConstants.stowPosition);
    }

    public boolean isSwitchPressed(){
        return !s2Signal.getValue();
    }

    @Override 
    public void periodic(){
        StatusSignal.refreshAll(s2Signal);

        //SmartDashboard.putNumber("Real Hood Angle", getHoodDegrees());

        boolean pressed = isSwitchPressed();
        if (pressed && !wasPressed) {
            zeroHood();
        }
        wasPressed = pressed;
    }
}
