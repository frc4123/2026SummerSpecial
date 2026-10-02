package frc.robot.commands.intakeRoller;

import org.wpilib.command2.Command;
import frc.robot.Constants.IntakeRollerConstants;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.IntakeRoller;

public class IntakeRollerIn extends Command{

    IntakeRoller intakeRollers;
    IntakeArm intakeArm;

    public IntakeRollerIn(IntakeRoller intakeRollers, IntakeArm intakeArm) {
        this.intakeRollers = intakeRollers;
        this.intakeArm = intakeArm;
        addRequirements(intakeRollers);
    }// TODO MAKE CHECK THE STATE OF THE ARM BEFORE ROLLING

    @Override
    public void execute() {
        if(intakeArm.getIntakePosition() <= 0.115) {
            intakeRollers.setIntakeVelo(IntakeRollerConstants.intakeVelo);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        intakeRollers.setIntakeVelo(IntakeRollerConstants.zeroVelo);
    }
}