package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class orbit extends SubsystemBase{
    public Command orbitRight(){
        return AutoBuilder.buildAuto("Orbit");
    }
}