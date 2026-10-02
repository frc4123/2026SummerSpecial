package frc.robot.commands.sevenEleven;

import org.wpilib.command2.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.subsystems.SevenEleven;

public class RollHigh extends Command{

    SevenEleven sevenEleven;

    public RollHigh(SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
        addRequirements((sevenEleven));
    }

    @Override
    public void execute() {
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.sevenElevenHighVelo);
    }
}