package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.subsystems.turret.TrajectoryCalculator.ShotData;
import frc.robot.utils.ShotCache;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.command2.SubsystemBase;


public class Uptake extends SubsystemBase{

    private final TalonFX uptakeMotor = new TalonFX(
        Constants.CanIdSystemCore.Uptake,
        Constants.CanIdSystemCore.canbus2
    );

    private final MotionMagicVelocityVoltage motionMagic =
        new MotionMagicVelocityVoltage(UptakeConstants.zeroVelo)
           .withVelocity(UptakeConstants.uptakeVelo)
           .withAcceleration(UptakeConstants.uptakeAcc
        );

    public Uptake(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        uptakeMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)));


        Slot0Configs pid = new Slot0Configs()
            .withKP(UptakeConstants.kP)
            .withKI(UptakeConstants.kI)
            .withKD(UptakeConstants.kD)
            .withKS(UptakeConstants.kS)
            .withKV(UptakeConstants.kV)
            .withKA(UptakeConstants.kA);

        uptakeMotor.getConfigurator().apply(pid);
    }

    public void zeroUptakeVelo(){
        uptakeMotor.setControl(motionMagic.withVelocity(0));
    }

    public void setUptakeVelo(double velo){
        ShotData shot = ShotCache.get();

        double Velo = shot.getExitVelocity().in(MetersPerSecond) * (2)
            / (2.0 * Math.PI * (ShooterConstants.flywheelRadius.in(Meters) + ShooterConstants.compression.in(Meters)));

            // THIS IS THE RATIO I DETERMIEND TO SHOOT FARTHER IF NEEDED IF IT MISSES SHOO
            // ShooterConstants.shootingTestErrorRatio; so multiply the final velo by that 

        uptakeMotor.setControl(motionMagic.withVelocity((Velo * ShooterConstants.shootingTestErrorRatio * 0.8)));  //1.23
    }

    public double getUptakeVelo() {
        return uptakeMotor.getVelocity().getValueAsDouble();
    }
}