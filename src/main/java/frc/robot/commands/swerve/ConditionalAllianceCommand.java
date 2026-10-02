package frc.robot.commands.swerve;

import org.wpilib.driverstation.MatchState;
//import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
// import org.wpilib.driverstation.MatchType;
// import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.command2.Command;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.ConditionalCommand;

public class ConditionalAllianceCommand extends SequentialCommandGroup {

    public ConditionalAllianceCommand(Command blueCmd, Command redCmd) {

        // Add conditional logic to choose commands
        addCommands(
            new ConditionalCommand(
                blueCmd,
                redCmd,
                () -> MatchState.getAlliance().get().equals(Alliance.BLUE)
            )
        );
    }
}