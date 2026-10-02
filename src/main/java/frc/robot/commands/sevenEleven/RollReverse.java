package frc.robot.commands.sevenEleven;

import org.wpilib.command2.Command;
import frc.robot.Constants.SevenElevenConstants;
import frc.robot.subsystems.SevenEleven;

public class RollReverse extends Command{

    SevenEleven sevenEleven;

    public RollReverse(SevenEleven sevenEleven) {
        this.sevenEleven = sevenEleven;
        addRequirements((sevenEleven));
    }

    @Override
    public void execute() {
        sevenEleven.setSevenElevenVelo(SevenElevenConstants.sevenElevenReverseVelo);
    }
}