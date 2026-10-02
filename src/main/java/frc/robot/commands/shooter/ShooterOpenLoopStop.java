package frc.robot.commands.shooter;

import org.wpilib.command2.Command;
import frc.robot.subsystems.Shooter;


public class ShooterOpenLoopStop extends Command {
    private final Shooter shooter;

    public ShooterOpenLoopStop(Shooter shooter){
        this.shooter= shooter;
        addRequirements(shooter);

    }

    @Override
    public void execute(){
        shooter.setShooterOpenLoopVelo(0);
    }
}

