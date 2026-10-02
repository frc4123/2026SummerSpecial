package frc.robot.commands.intakeRoller;

import org.wpilib.command2.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeRoller;

public class IntakeRollerStop extends Command{

    IntakeRoller intakeRollers;

    public IntakeRollerStop(IntakeRoller intakeRollers) {
        this.intakeRollers = intakeRollers;
        addRequirements(intakeRollers);
    }

    @Override
    public void execute() {
        intakeRollers.setIntakeVelo(IntakeRollerConstants.zeroVelo);
    }

}