package frc.robot.commands.climb;

import org.wpilib.command2.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbUp extends Command{

    Climb climb;

    public ClimbUp(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.setClimbPosition(ClimbConstants.upPosition);
    }
}