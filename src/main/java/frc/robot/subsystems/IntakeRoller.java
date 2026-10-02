package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConstants;

import static frc.robot.Constants.IntakeRollerConstants.intakeVelo;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command2.SubsystemBase;


public class IntakeRoller extends SubsystemBase{

    private final TalonFX intakeRollerMotor = new TalonFX(
        Constants.CanIdSystemCore.Intake_Roller,
        Constants.CanIdSystemCore.canbus2
    );
  
    // Motion Magic controller object

    private final MotionMagicVelocityVoltage motionMagic =
        new MotionMagicVelocityVoltage(IntakeRollerConstants.zeroVelo)
           .withVelocity(IntakeRollerConstants.intakeVelo)
           .withAcceleration(IntakeRollerConstants.intakeAcc
        );

    public IntakeRoller(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        intakeRollerMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));

    // intakeRollerMotor.setControl(new Follower(Constants.CanIdSystemCore.Intake_Roller, 
    //     MotorAlignmentValue.Aligned));


        Slot0Configs pid = new Slot0Configs()
            .withKP(IntakeRollerConstants.kP)
            .withKI(IntakeRollerConstants.kI)
            .withKD(IntakeRollerConstants.kD)
            .withKS(IntakeRollerConstants.kS)
            .withKV(IntakeRollerConstants.kV)
            .withKA(IntakeRollerConstants.kA);

        intakeRollerMotor.getConfigurator().apply(pid);
    }

    public void setIntakeVelo(double velo){
        intakeRollerMotor.setControl(  
            motionMagic.withVelocity(velo));
    }

    public boolean isIntaking() {
        return intakeRollerMotor.getVelocity().getValueAsDouble() > intakeVelo * 0.3;
    }

    public double getIntakeVelo() {
        return intakeRollerMotor.getVelocity().getValueAsDouble();
    }
}