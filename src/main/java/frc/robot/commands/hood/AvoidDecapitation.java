package frc.robot.commands.hood;

import org.wpilib.command2.Command;
import frc.robot.subsystems.Hood;


public class AvoidDecapitation extends Command {
    private final Hood hood;

    public AvoidDecapitation(Hood hood){
        this.hood = hood;
        addRequirements(hood);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void execute(){
        hood.lowerHood();
    }
}




