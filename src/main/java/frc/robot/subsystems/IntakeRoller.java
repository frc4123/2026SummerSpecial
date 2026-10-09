package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeRollerConstants;

import static frc.robot.Constants.IntakeRollerConstants.intakeVelo;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command2.SubsystemBase;


public class IntakeRoller extends SubsystemBase{

    private final TalonFX leftIntakeRollerMotor = new TalonFX(
        Constants.CanIdSystemCore.Left_Intake_Roller,
        Constants.CanIdSystemCore.canbus4
    );
    private final TalonFX rightIntakeRollerMotor = new TalonFX(
        Constants.CanIdSystemCore.Right_Intake_Roller,
        Constants.CanIdSystemCore.canbus4
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
        leftIntakeRollerMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));

        // rightIntakeRollerMotor.setControl(new Follower(Constants.CanIdSystemCore.Left_Intake_Roller, 
        //    MotorAlignmentValue.Aligned));


        Slot0Configs pid = new Slot0Configs()
            .withKP(IntakeRollerConstants.kP)
            .withKI(IntakeRollerConstants.kI)
            .withKD(IntakeRollerConstants.kD)
            .withKS(IntakeRollerConstants.kS)
            .withKV(IntakeRollerConstants.kV)
            .withKA(IntakeRollerConstants.kA);

        leftIntakeRollerMotor.getConfigurator().apply(pid);
        // rightIntakeRollerMotor.getConfigurator().apply(pid);
    }

    public void setIntakeVelo(double velo){
        leftIntakeRollerMotor.setControl(  
            motionMagic.withVelocity(velo));
    }

    public boolean isIntaking() {
        return leftIntakeRollerMotor.getVelocity().getValueAsDouble() > intakeVelo * 0.3;
    }

    public double getIntakeVelo() {
        return leftIntakeRollerMotor.getVelocity().getValueAsDouble();
    }
}