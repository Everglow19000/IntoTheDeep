

package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "MTCAFtontRight")
public class MTCAFtontRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FinalAutonomous autonumous = new FinalAutonomous();
        autonumous.firstCall(this, FinalAutonomous.StartPosition.FRONT_RIGHT);

        autonumous.SecondCall();
    }

}
