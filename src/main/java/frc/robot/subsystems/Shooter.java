package frc.robot.subsystems;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.turret.TrajectoryCalculator.ShotData;
import frc.robot.utils.ShotCache;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;
import org.wpilib.command2.SubsystemBase;


public class Shooter extends SubsystemBase{

    private final TalonFX shooterMotor = new TalonFX(
        Constants.CanIdSystemCore.Shooter,
        Constants.CanIdSystemCore.canbus4
    );

    private final VelocityTorqueCurrentFOC motionMagic =
        new VelocityTorqueCurrentFOC(ShooterConstants.MIN_SPEED.in(MetersPerSecond))
            .withAcceleration(ShooterConstants.acceleration
        );

    private final VelocityTorqueCurrentFOC slowMotionMagic =
        new VelocityTorqueCurrentFOC(ShooterConstants.MIN_SPEED.in(MetersPerSecond))
            .withAcceleration(ShooterConstants.slowAcceleration
        );

    public boolean isShooting = false;

    public String shootingString = "Shooting Slider";

    public double onTheGoSlider = 1.0;
    private final TunableDouble shootingSlider = Tunables.addDouble("Shooter/Shooting Slider", 1.0);

    public Shooter(){
        // τηισ ισ ωερυ ιμπορταντ
        configureMotor();
    }

    private void configureMotor() {
        shooterMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));


        Slot0Configs pid = new Slot0Configs()
            .withKP(ShooterConstants.kP)
            .withKI(ShooterConstants.kI)
            .withKD(ShooterConstants.kD)
            .withKS(ShooterConstants.kS)
            .withKV(ShooterConstants.kV)
            .withKA(ShooterConstants.kA);

        MotorOutputConfigs motorOutput = new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive);
        
        shooterMotor.getConfigurator().apply(pid);

        shooterMotor.getConfigurator().apply(motorOutput);
    }

    public void calculateShot() {

        ShotData shot = ShotCache.get();

        double Velo = shot.getExitVelocity().in(MetersPerSecond) * (2)
            / (2.0 * Math.PI * (ShooterConstants.flywheelRadius.in(Meters) + ShooterConstants.compression.in(Meters))) * onTheGoSlider * 0.97 ;//TODO IS .97 THE BEST 

            // THIS IS THE RATIO I DETERMIEND TO SHOOT FARTHER IF NEEDED IF IT MISSES SHOO
            // ShooterConstants.shootingTestErrorRatio; so multiply the final velo by that 

        if(isShooting){
            shooterMotor.setControl(motionMagic.withVelocity((Velo * ShooterConstants.shootingTestErrorRatio)));  //1.23
        } else {
            shooterMotor.setControl(slowMotionMagic.withVelocity((Velo * ShooterConstants.shootingTestErrorRatio)));  //1.23
        }
        
    }

    public void isShooting(boolean isShooting) {
        this.isShooting = isShooting;
    }

    public void shooterMinVelo() {
        shooterMotor.setControl(motionMagic.withVelocity(ShooterConstants.MIN_SPEED.in(MetersPerSecond)));
    }

    public void setShooterOpenLoopVelo(double velo) {
        shooterMotor.setThrottle(velo);
    }

    public double getShooterVelo() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }
    
    @Override
    public void periodic() {
        onTheGoSlider = shootingSlider.get();
     //SmartDashboard.putNumber("Shooting Slider", onTheGoSlider);
    }
}