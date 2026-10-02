package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;

public class threeBumpRight extends SubsystemBase{
    public Command threeBumpAuto(){
        return AutoBuilder.buildAuto("3BumpRight");
    }
}