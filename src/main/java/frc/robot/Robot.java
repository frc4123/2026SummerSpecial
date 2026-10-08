// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

// import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.networktables.NT4Publisher;

// import com.ctre.phoenix6.HootAutoReplay;

import org.wpilib.driverstation.MatchState;
// import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.internal.DriverStationBackend;
// import org.wpilib.driverstation.Alliance;
// import org.wpilib.driverstation.MatchType;
// import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.RobotController;
// import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.telemetry.Telemetry;
import frc.robot.utils.FuelSim;
import frc.robot.utils.ShiftHelpers;
import frc.robot.utils.ShotCache;
//import frc.robot.subsystems.Oculus;

public class Robot extends LoggedRobot {

    private Command m_autonomousCommand;
    private final RobotContainer m_robotContainer;

    private int dashboardCounter = 0;

    // /* log and replay timestamp and joystick data */
    // private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
    //     .withTimestampReplay()
    //     .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
        if(Constants.Sim.CURRENT_MODE == Constants.Sim.Mode.Sim) {
            Logger.addDataReceiver(new NT4Publisher());
            Logger.start();
        }
        Telemetry.log("Driver Checklist", 
            ">>   Brady   <<\n 1. Plug in ethernet until click\n2. Make sure controller inputs are in order\n3. Select appropriate auto\n\n>>   Milton & Joseph   <<\n1. Is Battery percentage above 30%\n2. Two PI's are plugged firmly into external battery\n3. Quest is plugged firmly into external battery\n4. Zero Intake\n5.  Zero Hood\n" );
        DriverStationBackend.silenceJoystickConnectionAlert(true);
    }

    @Override
    public void robotPeriodic() {
        //m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        ShotCache.update();

        // SmartDashboard.putNumber("Target Hood Angle", ShotCache.get().getHoodAngle().in(Degrees));
        // SmartDashboard.putNumber("Target Exit Velocity", ShotCache.get().getExitVelocity().in(MetersPerSecond));
        if (++dashboardCounter >= 25) {
            dashboardCounter = 0;
            Telemetry.log("Battery Voltage", RobotController.getBatteryVoltage());
            Telemetry.log("Match Data/MatchTime", MatchState.getMatchTime());
            Telemetry.log("Match Data/InShift", ShiftHelpers.currentShiftIsYours());
            Telemetry.log("Match Data/TimeLeftInShift", ShiftHelpers.timeLeftInShiftSeconds(MatchState.getMatchTime()));
            //Telemetry.log("Trust Quest", Oculus.trustQuest());
            ShiftHelpers.timeLeftInShiftSeconds(MatchState.getMatchTime());
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {}

    @Override
    public void simulationPeriodic() {
        FuelSim.getInstance().updateSim();
    }
}
