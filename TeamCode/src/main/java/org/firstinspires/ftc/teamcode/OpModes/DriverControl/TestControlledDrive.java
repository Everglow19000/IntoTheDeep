
package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import static org.apache.commons.math3.stat.StatUtils.max;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Disabled
@TeleOp(name = "TestControlledDrive", group = "main-drive")
public class TestControlledDrive extends LinearOpMode {

    public static double TILE_LENGTH = 60.5;

    /**
     * Creates a new Pose2d but with X and Y values multiplaied by Tile Length
     *
     * @param x The X value of the robot Position according to filed Axis - and in Tile units
     * @param y The Y value of the robot Position according to filed Axis - and in Tile units
     * @param Heading The angle of the Robot in Relation to the filed - 0 is North
     *
     * @return new Pose2d in CM Units
     */
    public Pose2d PoseInTiles(double x, double y, double Heading) {
        return new Pose2d(x * TILE_LENGTH, y * TILE_LENGTH, Heading);
    }

    private double chooseLain(Pose2d robotTileLocation) {
        double tileY = (int)robotTileLocation.getY() + 0.5;
        if((robotTileLocation.getX() > 3.5) && (tileY == 2.5)) tileY = 1.5;
        if((robotTileLocation.getX() > 3.5) && (tileY == 3.5)) tileY = 4.5;
        telemetry.addData("Target Lain ", tileY);
        return tileY;
    }

    private double anglePower(double angle) {
        double targetAngle = 0;
        if(angle >= PI/2) {
            targetAngle = PI;
        } else if(angle <= -PI/2){
            targetAngle = -PI;
        }
        if(abs(targetAngle - angle) < 3.0 / 180.0 * PI) return 0;
        return (targetAngle - angle) * 3;
    }

    private Pose2d PassPowers(Pose2d robotTileLocation, Pose2d axisPowers) {
        final double X = robotTileLocation.getX(), Y = robotTileLocation.getY();
        double deviationX = 0;
        if(X > 3.5) deviationX = X - 3.5;
        if(X < 1.5) deviationX = 1.5 - X;

        double deviationY = chooseLain(robotTileLocation) - Y;

        double Py = 0; // deviationY * 0.4
        double Pr = 0;
        if(deviationX != 0){
            Py = deviationY * abs(deviationY / deviationX * axisPowers.getX() * 5);
            Pr = anglePower(robotTileLocation.getHeading());
        }

        return new Pose2d(axisPowers.getX(), Py, Pr);
        //return new Pose2d(axisPowers.getX(), Py, axisPowers.getHeading());
    }

    public static Pose2d driveByAxis(Pose2d inputPowers, double heading) {
        double powerX =  inputPowers.getX() * cos(heading) + inputPowers.getY() * sin(heading);
        double powerY =  -inputPowers.getX() * sin(heading) + inputPowers.getY() * cos(heading);
        return new Pose2d(powerX, powerY, inputPowers.getHeading());
    }


    Pose2d controlledDriving(Pose2d robotTileLocation, Pose2d inputPowers) {
        Pose2d axisPowers = driveByAxis(inputPowers, robotTileLocation.getHeading());
        telemetry.addData("axisPowers ", axisPowers);
        final double X = robotTileLocation.getX();
        if(X > 4 && axisPowers.getX() > 0.1 + (5.0 - X) / 4) {
            return new Pose2d(0.1 + (5.0 - X) / 4, axisPowers.getY(), axisPowers.getHeading());
        }

        if(X > 4 || X < 1) return axisPowers;
        if((X > 3.5 && axisPowers.getX() >= 0) || (X < 1.5 && axisPowers.getX() <= 0)) return axisPowers;
        if(axisPowers.getY() > 0.6) {
            return new Pose2d(0, axisPowers.getY(), axisPowers.getHeading());
        }

        return PassPowers(robotTileLocation, inputPowers);
    }

    public Pose2d locationInTiles() {
        Pose2d pos = drive.getPoseEstimate();
        return new Pose2d(pos.getX() / TILE_LENGTH, pos.getY() / TILE_LENGTH, realAngle(pos.getHeading()));
    }

    SampleMecanumDrive drive;

    public static double realAngle(double angle) {
        if(angle > PI) angle -= 2 * PI;
        return angle;
    }

    public static Pose2d adjustedPowers(Pose2d Powers) {
        return new Pose2d (Powers.getX() * 1.0, Powers.getY() * 1.13, Powers.getHeading() * 1.0);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(PoseInTiles(4, 5, 0));

        waitForStart();
        Pose2d lastLocation=locationInTiles();
        double maxDeltaPose=0;
        while(opModeIsActive()) {
            double Px = -gamepad1.left_stick_y,
                   Py = -gamepad1.left_stick_x,
                   Pangle = -gamepad1.right_stick_x;
            if(gamepad1.left_stick_button) {
                Px /= 3.0;
                Py /= 3.0;
            }
            if(gamepad1.right_stick_button) {
                Pangle /= 5.0;
            }
            Pose2d powers = new Pose2d(Px, Py, Pangle);


            Pose2d currentLocation = locationInTiles();
            double deltaPose = sqrt(
                    pow((currentLocation.getX()-lastLocation.getX()),2)
                            +pow((currentLocation.getY()-lastLocation.getY()),2));
            maxDeltaPose=Math.max(maxDeltaPose,deltaPose);
            telemetry.addData("Location ", currentLocation);
            telemetry.addData("powers ", powers);
            telemetry.addData("deltaPose in tiles", maxDeltaPose);

            //Pose2d controlledPowers = driveByAxis(powers, currentLocation.getHeading());
            Pose2d controlledPowers = controlledDriving(currentLocation, powers);
            telemetry.addData("controlledPowers", controlledPowers);
            drive.setWeightedDrivePower(adjustedPowers(controlledPowers));
            telemetry.update();
            drive.update();

            lastLocation=currentLocation;
        }
    }
}
