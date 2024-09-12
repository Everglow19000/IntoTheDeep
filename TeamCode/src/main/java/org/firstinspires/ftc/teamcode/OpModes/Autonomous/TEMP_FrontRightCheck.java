package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.EverglowLibrary.Systems.CameraSystem;
import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(group = "test", name = "TEMP_FrontRightCheck")
public class TEMP_FrontRightCheck extends LinearOpMode {
    SampleMecanumDrive drive;
    ElevatorSystem elevatorSystem;
    ClawSystem clawSystem;
    FourBarSystem fourBarSystem;
    GWheelSystem gWheelSystem;
    static double squareSize = 60.5; //in cm
    static double distanceBetweenTags=15.0; //in cm
    @Override
    public void runOpMode() throws InterruptedException {
        CameraSystem cameraSystem = new CameraSystem(this);
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevatorSystem = new ElevatorSystem(this);
        clawSystem = new ClawSystem(this);
        fourBarSystem = new FourBarSystem(this);
        gWheelSystem = new GWheelSystem(this);

        final double North = 0;
        final double East = -PI/2;
        final double West = PI/2;
        final double South = PI;

        waitForStart();
        //drive(startLocation, FirstDropLocation);
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(squareSize * 5.5, squareSize * 1.5, East))
                .splineTo(new Vector2d(4.5 * squareSize, squareSize * 2.5),South)
                .build();
        drive.followTrajectory(traj1);
        sleep(1000);

        //להפעיל גג"ז
        gWheelSystem.toggle(true);
        sleep(2000);
        gWheelSystem.toggle(true);
        sleep(1000);

        //drive(FirstDropLocation, SecondDropMiddlePoint); drive(SecondDropMiddlePoint, SecondDropLocation);
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(4.5 * squareSize, 2.5 * squareSize),South)
                .splineTo(new Vector2d(4.5 * squareSize +distanceBetweenTags,5 * squareSize),North)
                .build();
        drive.followTrajectory(traj2);
        sleep(1000);

        //פריקה שנייה
        elevatorSystem.goTo(ElevatorSystem.Level.UP);
        fourBarSystem.setServoPositionByLevel(FourBarSystem.ServoAngel.PICKUP);
        fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.PICKUP);
        sleep(2000);
        clawSystem.toggle();
        sleep(1000);

        //drive(SecondDropLocation,SecondDropLocation - 20)
        drive.followTrajectory(drive.trajectoryBuilder(traj2.end())
                .forward(-20)
                .build());
        fourBarSystem.setServoPositionByLevel(FourBarSystem.ServoAngel.PICKUP);
        sleep(1000);
        fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.START);

    }
}
