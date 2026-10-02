package frc.robot.commands.sevenEleven;

import org.wpilib.command2.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.subsystems.SevenEleven;

public class RollMid extends Command{

    SevenEleven sevenEleven;

    public RollMid(SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
        addRequirements((sevenEleven));
    }

    @Override
    public void execute() {
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.sevenElevenMidVelo);
    }
}