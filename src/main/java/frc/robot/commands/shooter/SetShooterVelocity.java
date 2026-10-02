package frc.robot.commands.shooter;

import org.wpilib.command2.Command;
import frc.robot.subsystems.Shooter;


public class SetShooterVelocity extends Command {
    private final Shooter shooter;

    public SetShooterVelocity(Shooter shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    @Override
    public void execute(){
        shooter.calculateShot();
    }
}




