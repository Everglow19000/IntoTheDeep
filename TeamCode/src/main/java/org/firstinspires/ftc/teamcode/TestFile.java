package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.jetbrains.annotations.NotNull;

public class TestFile {
    public class TestOpMode extends LinearOpMode {
        private DcMotorEx motor;
        @Override
        public void runOpMode() throws InterruptedException {
            motor = hardwareMap.get(DcMotorEx.class, "RightLeft");

            waitForStart();

            motor.setPower(0.8);
            double vel = motor.getVelocity();
            System.out.println(vel);
        }
    }
}
