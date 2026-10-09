package frc.robot.commands.uptake;

import org.wpilib.command2.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.Constants.UptakeConstants;
import frc.robot.subsystems.SevenEleven;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Uptake;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utils.ShotHelper;

public class UptakeUp extends Command{

    Uptake uptake;
    Turret turret;
    SevenEleven sevenEleven;
    Shooter shooter;

    public UptakeUp(Uptake uptake, Turret turret, SevenEleven sevenEleven, Shooter shooter){ 
        this.uptake = uptake;
        this.turret = turret;
        this.sevenEleven = sevenEleven;
        this.shooter = shooter;
        addRequirements(uptake, sevenEleven);
    }

    @Override
    public void execute() {
        if (ShotHelper.getIsWrapping()){
            uptake.zeroUptakeVelo();
            return;
        }
        shooter.isShooting(true);
        uptake.setUptakeVelo(UptakeConstants.uptakeVelo);
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.sevenElevenHighVelo);
    }
    
    @Override
    public void end(boolean interrupted) {
        uptake.zeroUptakeVelo();
        shooter.isShooting(false);
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.zeroVelo);
    }
}