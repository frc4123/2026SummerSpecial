package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class CityBoyRight extends SubsystemBase{
    public Command cityBoyRight(){
        return AutoBuilder.buildAuto("CityBoyRight");
    }
}