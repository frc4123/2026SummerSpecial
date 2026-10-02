package frc.robot.commands.climb;

import org.wpilib.command2.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;

public class ClimbTest extends Command{

    Climb climb;

    public ClimbTest(Climb climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.setClimbPosition(ClimbConstants.testPosition);
    }
}