package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class twoCycle extends SubsystemBase{
    public Command twoCycleClimb(){
        return AutoBuilder.buildAuto("2CycleRight");
    }
}