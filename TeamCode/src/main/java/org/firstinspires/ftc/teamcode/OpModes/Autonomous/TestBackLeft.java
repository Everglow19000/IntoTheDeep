package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "test")
public class TestBackLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FinalAutonomous fourtyFivePoints = new FinalAutonomous();
        fourtyFivePoints.firstCall(this, FinalAutonomous.StartPosition.BACK_LEFT);
        fourtyFivePoints.SecondCall();
    }
}
