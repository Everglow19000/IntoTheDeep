package org.firstinspires.ftc.teamcode.OpModes.Autonomous.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.FinalAutonomous;

@Autonomous(group = "test")
public class BackLeft_R extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FinalAutonomous finalAutonomous = new FinalAutonomous();
        finalAutonomous.firstCall(this, FinalAutonomous.StartPosition.FRONT_RIGHT);
        if(isStopRequested())
            return;
        finalAutonomous.runAfterInputTest(CameraSystem.DetectionLocation.RIGHT);
        finalAutonomous.SecondCall();
    }
}
