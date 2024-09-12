
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "MTCABackLeft")
public class MTCABackLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FinalAutonomous autonumous = new FinalAutonomous();
        autonumous.firstCall(this, FinalAutonomous.StartPosition.BACK_LEFT);

        autonumous.SecondCall();
    }
}
