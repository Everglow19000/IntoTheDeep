package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

public class PracticeFile {
    public class TestOpMode extends LinearOpMode {
        private DcMotorEx motor;
        private Gyroscope imu;
        private DigitalChannel digitalTouch;
        private DistanceSensor sensorColorRange;
        private Servo servoTest;
        @Override
        public void runOpMode() throws InterruptedException {
            motor = hardwareMap.get(DcMotorEx.class, "RightLeft");
            imu = hardwareMap.get(Gyroscope.class, "imu");
            digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
            sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
            servoTest = hardwareMap.get(Servo.class, "servoTest");

            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            motor.setPower(0.8);
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
        }
    }
}















/*
        double tgtPower = 0;
        tgtPower = -this.gamepad1.left_stick_y;
        motor.setPower(tgtPower);
        telemetry.addData("Target Power", tgtPower);
        telemetry.addData("Motor Power", motor.getPower());*/
