package frc.robot.commands.hood;

import org.wpilib.command2.Command;
import frc.robot.subsystems.Hood;
import frc.robot.utils.DecapitationHelper;


public class HoodAim extends Command {
    private final Hood hood;

    public HoodAim(Hood hood){
        this.hood = hood;
        addRequirements(hood);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void execute(){
        if(DecapitationHelper.closeToTrench()) {
            hood.lowerHood();
            return;
        }
        
        hood.setHoodAngle();
    }
}




