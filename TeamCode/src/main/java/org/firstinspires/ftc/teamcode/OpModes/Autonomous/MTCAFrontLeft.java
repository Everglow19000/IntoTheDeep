package org.firstinspires.ftc.teamcode.OpModes.Autonomous;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "MTCAFrontLeft")
public class MTCAFrontLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FinalAutonomous autonumous = new FinalAutonomous();
        autonumous.firstCall(this, FinalAutonomous.StartPosition.FRONT_LEFT);

        autonumous.SecondCall();
    }
}
