package frc.robot.commands.sevenEleven;

import org.wpilib.command2.Command;
import frc.robot.subsystems.SevenEleven;

public class RollStop extends Command{

    SevenEleven sevenEleven;

    public RollStop(SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
        addRequirements((sevenEleven));
    }

    @Override
    public void execute() {
        sevenEleven.setSevenElevenVelo(0);
    }
}