package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.EverglowLibrary.Systems.CameraSystem;

import static java.lang.Math.abs;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.EverglowLibrary.ThreadHandleLib.SequenceRunner;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class FinalAutonomous {

    ///////////////////////////////
    ///////////////////////////////
    // Start Position functions  //
    ///////////////////////////////
    ///////////////////////////////

    public enum StartPosition{
        FRONT_LEFT(new Vector2d(0.25, 3.75)), FRONT_RIGHT(new Vector2d(5.75, 3.75))
        , BACK_LEFT(new Vector2d(0.25, 1.75)), BACK_RIGHT(new Vector2d(5.75, 1.75));

        public final Vector2d startPositionVector;
        StartPosition(Vector2d startPosition) {
            this.startPositionVector = startPosition;
        }

        public boolean isRight() {
            if(this == FRONT_RIGHT || this == BACK_RIGHT)
                return true;
            else
                return false;
        }

        public boolean isBack() {
            if(this == BACK_LEFT || this == BACK_RIGHT)
                return true;
            else
                return false;
        }
    }


    /**
     * Does the robot start on the right(Red Alliance)
     *
     * @return 0 - start on left, 1 - starts on right
     */
    public boolean isRight() { return startPosition.isRight(); }


    /**
     * Does the robot start in the back of the filed
     *
     * @return 0 - start in Front, 1 - starts in the Back
     */
    public boolean isBack() { return startPosition.isBack(); }


    public double trueAngle(double angel) {
        if(angel > PI /2) {
            angel -= PI;
        }
        else if(angel < PI /2) {
            angel += PI;
        }
        return angel;
    }

    private void turnTo(double angel) {
        double currentAngle = LocationInTiles().getHeading();
        double dev = trueAngle(angel - currentAngle);
        drive.turn(dev);
    }


    /////////////////////////////////////////////
    /////////////////////////////////////////////
    // Useful Functions That Manipulate Pose2d //
    /////////////////////////////////////////////
    /////////////////////////////////////////////

    /**
     * Creates a new Pose2d but with X and Y values multiplaied by Tile Length
     *
     * @param x The X value of the robot Position according to filed Axis - and in Tile units
     * @param y The Y value of the robot Position according to filed Axis - and in Tile units
     * @param Heading The angle of the Robot in Relation to the filed - 0 is North
     *
     * @return new Pose2d in CM Units
     */
    public static Pose2d PoseInTiles(double x, double y, double Heading) {
        return new Pose2d(x * TILE_LENGTH, y * TILE_LENGTH, Heading);
    }

    public Pose2d LocationInTiles() {
        Pose2d location = drive.getPoseEstimate();
        return new Pose2d(location.getX() / TILE_LENGTH, location.getY() / TILE_LENGTH, trueAngle(Math.toRadians(location.getY())));
    }


    /**
     * Mirrors any Pose2d from the left side to the right side
     * Use to turn any Pose that is written for a Left side Autonomous to work for Right side Autonomous
     *
     * @param pose the Location of the Robot on the Filed
     * @return new Pose2d to use for a Right-Start Autonomous
     */
    public Pose2d mirrorToRight(Pose2d pose) {
        return new Pose2d(pose.getX(), 6 * TILE_LENGTH - pose.getY(), -pose.getHeading());
    }

    public Pose2d mirrorToFront(Pose2d pose) {
        return new Pose2d(6 * TILE_LENGTH - pose.getX(),  pose.getY(), PI - pose.getHeading());
    }


    /**
     * Makes any Pose2d be Correct for Both Left(Blue Alliance) and Right(red Alliance) Autonomous
     * by Mirroring to the right - if and only if needed - according to the startPosition.
     *
     * @param pose the Location of the Robot on the Filed
     * @return new Pose2d to use
     */
    public Pose2d tryRight(Pose2d pose) {
        if(isRight() && pose.getY() > 3 * TILE_LENGTH) {
            pose = mirrorToRight(pose);
        }
        return pose;
    }

    public Pose2d tryFront(Pose2d pose){
        if(!isBack() && pose.getX() < 3 * TILE_LENGTH) {
            pose = mirrorToFront(pose);
        }
        return pose;
    }


    /**
     * Mirrors any Pose2d using the trust as the mirror line, for nerow use to copy from a Front Autonomous to a Back Autonomous
     *
     * @param pose the Location of the Robot on the Filed, written for a Front Autonomous
     * @return new Pose2d Mirrored to the Trust
     */
    public Pose2d mirrorToTrust(Pose2d pose) {
        final double middleOfTrust = 2.5 * TILE_LENGTH;
        if(pose.getHeading() >= 0) return new Pose2d(2 * middleOfTrust - pose.getX(), pose.getY(), PI - pose.getHeading());
        return new Pose2d(2 * middleOfTrust - pose.getX(), pose.getY(), -PI - pose.getHeading());
    }

    /////////////
    /////////////
    // Systems //
    /////////////
    /////////////

    LinearOpMode opMode;
    SequenceControl sequenceControl;
    SampleMecanumDrive drive;
    ElevatorSystem elevatorSystem;
    ClawSystem clawSystem;
    FourBarSystem fourBarSystem;
    GWheelSystem gWheelSystem;

    CameraSystem cameraSystem;

    SequenceRunner sequenceRunner = new SequenceRunner();

    ///////////////
    ///////////////
    // Constants //
    ///////////////
    ///////////////

    static double distanceOfPropFromRobot = 67; //in cm
    static double distanceBetweenTags = 13.5; //in cm
    static double distanceBuffer=0;

    static final double North = 0;
    static final double East = -PI/2;
    static final double West = PI/2;
    static final double South = PI;

    static final double TILE_LENGTH = 60; // 60.5


    ///////////////////////////
    ///////////////////////////
    // Autonomous Parameters //
    ///////////////////////////
    ///////////////////////////

    Sequence getReadyToDrop, returnSystemsToStart, returnFromPurple;
    StartPosition startPosition = StartPosition.FRONT_LEFT;
    CameraSystem.DetectionLocation propPlace = CameraSystem.DetectionLocation.RIGHT;

    Trajectory splineToPurple, trajMiddleForBack;

    Trajectory tempPark;
    ThreeTrajectories threePurpleDropTrajectories;

    ThreeTrajectories threeYellowDropTrajectories;
    ThreeTrajectories threeParkTrajectories;
    ThreeTrajectories threeMiddleForBackTrajectories;

    Sequence dropPurpleSeq;

    Sequence dropYellow;

    Pose2d StartLocation;



    ////////////////////
    ////////////////////
    // Main Functions //
    ////////////////////
    ////////////////////

    public void firstCall(LinearOpMode opMode, StartPosition startPosition) throws InterruptedException{
        // Initalize Systems //
        this.opMode = opMode;
        this.startPosition = startPosition;
        drive = new SampleMecanumDrive(opMode.hardwareMap);
        Servo planeServo = opMode.hardwareMap.get(Servo.class, "PlaneServo");
        planeServo.setPosition(0);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cameraSystem = new CameraSystem(opMode, isRight(), isBack());


        elevatorSystem = new ElevatorSystem(opMode);
        clawSystem = new ClawSystem(opMode);
        fourBarSystem = new FourBarSystem(opMode);
        gWheelSystem = new GWheelSystem(opMode);

        opMode.sleep(1000);
        fourBarSystem.restart();

        sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        sequenceRunner = new SequenceRunner();
        threeParkTrajectories = new ThreeTrajectories(drive);
        threePurpleDropTrajectories = new ThreeTrajectories(drive);
        threeYellowDropTrajectories = new ThreeTrajectories(drive);
        threeMiddleForBackTrajectories = new ThreeTrajectories(drive);

        // Create all Sequences //
        dropPurpleSeq  = new Sequence(false,
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                //fourBarSystem.getExecutor(FourBarSystem.ServoAngel.DROP, true),
                fourBarSystem.getExecutor(FourBarSystem.Level.LOW, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)
        );

        dropYellow =  new Sequence(false, clawSystem.getExecutor(false)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                ,fourBarSystem.getExecutor(FourBarSystem.ServoAngel.DROP, true)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP)
                , elevatorSystem.getExecutor(ElevatorSystem.Level.MED_AUTONOMOUS)
        );

        getReadyToDrop  = sequenceControl.GetReadyToDropSeq();
        returnSystemsToStart = sequenceControl.DropAndRetreatSeq();
        returnFromPurple = sequenceControl.DropAndRetreatSeq();


        // Trajectory Calculations //

        StartLocation = PoseInTiles(3.71, 5.6, East);

        if(isBack()) {
            opMode.telemetry.addLine("move the pose...");
            StartLocation = StartLocation.minus(PoseInTiles(2.42, 0, 0));
        }

        StartLocation = tryRight(StartLocation);
        drive.setPoseEstimate(StartLocation);

        // Complex drive to Purple //

        threePurpleDropTrajectories.createThreePoseStart(new ThreePose(StartLocation));

        ThreePose parkLocation;
        double correctPark;
        double rightBuffer = 0;
        double multy = 0;
        ThreePose purpleDropLocation;
        int signX = getXSignByPosition(), signY = getYSignByPosition();

        // drop of the purple in the back //
        if((!isRight() && !isBack()) || (isRight() && isBack())){
            purpleDropLocation = new ThreePose(
                    tryRight(tryFront(PoseInTiles(1, 4.01, South))), //middle
                    tryRight(tryFront(PoseInTiles(1.52, 4.3, North))), //left
                    tryRight(tryFront(PoseInTiles(1.382, 4.3, South))) //right
            );
        }
        else{
            purpleDropLocation = new ThreePose(
                    tryRight(tryFront(PoseInTiles(1, 4.01, South))), //middle
                    tryRight(tryFront(PoseInTiles(1.382, 4.3, South))), //left
                    tryRight(tryFront(PoseInTiles(1.52, 4.3, North))) //right
            );
        }

        Pose2d purpleHalfWayLocation = tryFront(tryRight(PoseInTiles(0.5,5,South)));
        double frontRightBuff = 0, frontLeftBuff = 0;
        if(isBack()) {
            rightBuffer = 0.5;
            ThreePose waitLoc;
            if (isRight()) {
                multy = 2;
            }

            //rightBuffer = 0.35;

            opMode.telemetry.addData("purple right: ", purpleDropLocation.poseRight);
            opMode.telemetry.addData("purple Halfway: ", purpleHalfWayLocation);

            ThreePose between = new ThreePose(
                purpleDropLocation.poseMiddle.plus(tryRight(PoseInTiles(
                        0,3.65,0))
                        .plus(PoseInTiles(0,-purpleDropLocation.poseMiddle.getY()/TILE_LENGTH,0))),
                purpleDropLocation.poseLeft.plus(tryRight(PoseInTiles(
                        0,3.65,0))
                        .plus(PoseInTiles(0,-purpleDropLocation.poseLeft.getY()/TILE_LENGTH,0))),
                purpleDropLocation.poseRight.plus(tryRight(PoseInTiles(
                        -0.2 * signX,3.65,0))
                        .plus(PoseInTiles(0,-purpleDropLocation.poseRight.getY()/TILE_LENGTH,0)))
            );

            threeMiddleForBackTrajectories.createThreePoseStart(purpleDropLocation, true);
            threeMiddleForBackTrajectories.addLineToSplineHeading(between);

            //waitLoc.poseRight = waitLoc.poseRight.plus(tryRight(PoseInTiles(0,rightBuffer,0))); //rightBuff
            //waitLoc.poseMiddle = waitLoc.poseMiddle.plus(PoseInTiles(0,-backRightBuff,0)); //-backRightBuff
            //waitLoc.poseLeft = waitLoc.poseLeft.plus(PoseInTiles(0,-backRightBuff,0)); //-backRightBuff
            waitLoc = new ThreePose(
                    tryRight(PoseInTiles(3, 3.25, South)),
                    tryRight(PoseInTiles(3, 3.25, South)),
                    tryRight(PoseInTiles(3, 3.25, North))
                            .plus(PoseInTiles(0, rightBuffer * signY,0)));

            waitLoc = createRightPose(waitLoc);

            threeMiddleForBackTrajectories.addConstHeadingTraj(waitLoc);

            waitLoc = new ThreePose(
                    tryRight(PoseInTiles(4.45, 3.25, South)),
                    tryRight(PoseInTiles(4.45, 3.25,South)),
                    tryRight(PoseInTiles(4.45, 3.25, North))
                            .plus(PoseInTiles(0, rightBuffer * signY,0)));

            waitLoc = createRightPose(waitLoc);

            threeMiddleForBackTrajectories.addConstHeadingTraj(waitLoc);

            waitLoc = new ThreePose(
                    tryRight(PoseInTiles(4.45, 3.23, South)),
                    tryRight(PoseInTiles(4.45, 3.23,South)),
                    tryRight(PoseInTiles(4.45, 3.23, South))
                            .plus(PoseInTiles(0, rightBuffer * signY,0)));

            waitLoc = createRightPose(waitLoc);

            if(isRight()){
                threeMiddleForBackTrajectories.trajbuilderLeft
                        .splineTo(waitLoc.poseLeft.vec(), waitLoc.poseLeft.getHeading());
                threeMiddleForBackTrajectories.trajbuilderRight
                        .splineToConstantHeading(waitLoc.poseRight.vec(),waitLoc.poseRight.getHeading());
            }else {
                threeMiddleForBackTrajectories.trajbuilderLeft
                        .splineToConstantHeading(waitLoc.poseLeft.vec(), waitLoc.poseLeft.getHeading());
                threeMiddleForBackTrajectories.trajbuilderRight
                        .splineTo(waitLoc.poseRight.vec(),waitLoc.poseRight.getHeading());
            }
            threeMiddleForBackTrajectories.trajbuilderMiddle
                    .splineToConstantHeading(waitLoc.poseMiddle.vec(), waitLoc.poseMiddle.getHeading());
            threeMiddleForBackTrajectories.endLocations = waitLoc;
            correctPark = 3.5;
            threeYellowDropTrajectories.startLocations = threeMiddleForBackTrajectories.endLocations;
            threeMiddleForBackTrajectories.buildTrajs();
        }
        else{
            purpleDropLocation.poseLeft = purpleDropLocation.poseLeft.plus(PoseInTiles(-1,0,0));
            purpleDropLocation.poseMiddle = purpleDropLocation.poseMiddle.plus(PoseInTiles(-1,0,0));
            purpleDropLocation.poseRight = purpleDropLocation.poseRight.plus(PoseInTiles(-1,0,0));

            if(isRight()){
                purpleDropLocation.poseRight = purpleDropLocation.poseRight.plus(PoseInTiles(1.175,0,PI));
            }
            else {
                purpleDropLocation.poseLeft = purpleDropLocation.poseLeft.plus(PoseInTiles(1.175,0,PI));
            }

            purpleHalfWayLocation = purpleHalfWayLocation.plus(PoseInTiles(-1,0,0));

            correctPark = 5.6;
            if(isRight())
                frontRightBuff = 0.07;
            else
                frontLeftBuff = 0.07;
//            angleFront = 0;
            threePurpleDropTrajectories.endLocations = purpleDropLocation;
            threeYellowDropTrajectories.startLocations = threePurpleDropTrajectories.endLocations;
        }
        // purple drop location //

        if((isBack() && !isRight()) || (!isBack() && isRight())) {
            threePurpleDropTrajectories.trajbuilderLeft
                    .lineToSplineHeading(purpleHalfWayLocation)
                    .splineToConstantHeading(purpleDropLocation.poseLeft.vec(), purpleDropLocation.poseLeft.getHeading());

            threePurpleDropTrajectories.trajbuilderRight
                    .splineToLinearHeading(purpleDropLocation.poseRight, purpleDropLocation.poseRight.getHeading());

        }
        else{
            threePurpleDropTrajectories.trajbuilderRight
                    .lineToSplineHeading(purpleHalfWayLocation)
                    .splineToConstantHeading(purpleDropLocation.poseRight.vec(), purpleDropLocation.poseRight.getHeading());
            threePurpleDropTrajectories.trajbuilderLeft
                    .splineToLinearHeading(purpleDropLocation.poseLeft, purpleDropLocation.poseLeft.getHeading());
        }

        threePurpleDropTrajectories.trajbuilderMiddle
                .lineToSplineHeading(purpleHalfWayLocation)
                .splineToConstantHeading(purpleDropLocation.poseMiddle.vec(), purpleDropLocation.poseMiddle.getHeading());
        threePurpleDropTrajectories.buildTrajs();

        // end of purple drop location //

        // yellow Trajectories //
        threeYellowDropTrajectories.endLocations.poseMiddle =
                tryRight(PoseInTiles(4.9, 4.5, South)); //backRightBuff, angleFront 4.4
        threeYellowDropTrajectories.endLocations.poseLeft =
                threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                        new Pose2d(frontLeftBuff*TILE_LENGTH, distanceBetweenTags, 0));
        threeYellowDropTrajectories.endLocations.poseRight
                = threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                new Pose2d(frontRightBuff*TILE_LENGTH, -distanceBetweenTags, 0)); //where is the 0 was (rightBuffer * TILE_LENGTH)

        if(!isBack())
            threeYellowDropTrajectories.createLinerHeadingTrajectories(true);
        else
            threeYellowDropTrajectories.createConstHeadingTrajectories(true);

        // park //
        ThreePose dropYellowLocs = threeYellowDropTrajectories.endLocations;

        ThreePose between = new ThreePose(
                dropYellowLocs.poseMiddle.plus(tryRight(PoseInTiles(
                        -0.15,correctPark,0)))
                        .plus(PoseInTiles(0,-dropYellowLocs.poseMiddle.getY()/TILE_LENGTH,0)),
                dropYellowLocs.poseLeft.plus(tryRight(PoseInTiles(
                        -0.15,correctPark,0)))
                        .plus(PoseInTiles(0,-dropYellowLocs.poseLeft.getY()/TILE_LENGTH,0)),
                dropYellowLocs.poseRight.plus(tryRight(PoseInTiles(
                        -0.15,correctPark,0)))
                        .plus(PoseInTiles(0,-dropYellowLocs.poseRight.getY()/TILE_LENGTH,0))
        );

        threeParkTrajectories.createThreePoseStart(threeYellowDropTrajectories.endLocations, true);
        threeParkTrajectories.addConstHeadingTraj(between);

        parkLocation = new ThreePose(
                between.poseMiddle.plus(new Pose2d(TILE_LENGTH/1.6, 0, 0)),
                between.poseLeft.plus(new Pose2d(TILE_LENGTH/1.6,0,0)),
                between.poseRight.plus(new Pose2d(TILE_LENGTH/1.6,0,0)));

        threeParkTrajectories.addConstHeadingTraj(parkLocation);
        threeParkTrajectories.buildTrajs();

        clawSystem.MoveOneClaw(true, false);

        opMode.telemetry.addLine("Ready!!!!! ");
        opMode.telemetry.update();

        opMode.waitForStart();

        if(opMode.isStopRequested())
            return;

        runAfterInput();
    }

    public ThreePose createRightPose(ThreePose threePose){
        if(isRight()){
            threePose = new ThreePose(
                    threePose.poseMiddle,
                    threePose.poseRight,
                    threePose.poseLeft
            );
        }
        return threePose;
    }

    public void BuildAutonomous(LinearOpMode opMode, StartPosition startPosition){
        this.opMode = opMode;
        this.startPosition = startPosition;
        drive = new SampleMecanumDrive(opMode.hardwareMap);
        Servo planeServo = opMode.hardwareMap.get(Servo.class, "PlaneServo");
        planeServo.setPosition(0);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cameraSystem = new CameraSystem(opMode, isRight(), isBack());


        elevatorSystem = new ElevatorSystem(opMode);
        clawSystem = new ClawSystem(opMode);
        fourBarSystem = new FourBarSystem(opMode);
        gWheelSystem = new GWheelSystem(opMode);

        opMode.sleep(1000);
        fourBarSystem.restart();

        sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        sequenceRunner = new SequenceRunner();
        threeParkTrajectories = new ThreeTrajectories(drive);
        threePurpleDropTrajectories = new ThreeTrajectories(drive);
        threeYellowDropTrajectories = new ThreeTrajectories(drive);
        threeMiddleForBackTrajectories = new ThreeTrajectories(drive);

        // Create all Sequences //
        dropPurpleSeq  = new Sequence(false,
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.LOW, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)
        );

        dropYellow = new Sequence(false, clawSystem.getExecutor(false)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.MED_AUTONOMOUS)
        );

        getReadyToDrop  = sequenceControl.GetReadyToDropSeq();
        returnSystemsToStart = sequenceControl.DropAndRetreatSeq();
        returnFromPurple = sequenceControl.DropAndRetreatSeq();

        switch (startPosition){
            case BACK_LEFT: {
                StartLocation = PoseInTiles(1.29, 5.6, East);
                drive.setPoseEstimate(StartLocation);
                threePurpleDropTrajectories.createThreePoseStart(new ThreePose(StartLocation));

                ThreePose purpleDropLocation = new ThreePose(
                        PoseInTiles(1.1, 4.01, South), //middle
                        PoseInTiles(1.383, 4.3, South), //left
                        PoseInTiles(1.52, 4.3, North) //right
                );

                Pose2d purpleHalfWayLocation = PoseInTiles(0.5,5,South);

                threePurpleDropTrajectories.trajbuilderLeft //add path to the left trajectory
                        .lineToSplineHeading(purpleHalfWayLocation)
                        .splineToConstantHeading(purpleDropLocation.poseLeft.vec(), purpleDropLocation.poseLeft.getHeading());

                threePurpleDropTrajectories.trajbuilderRight
                        .splineToLinearHeading(purpleDropLocation.poseRight, purpleDropLocation.poseRight.getHeading());

                threePurpleDropTrajectories.trajbuilderMiddle
                        .lineToSplineHeading(purpleHalfWayLocation)
                        .splineToConstantHeading(purpleDropLocation.poseMiddle.vec(), purpleDropLocation.poseMiddle.getHeading());

                threePurpleDropTrajectories.buildTrajs();
                break;
            }
            case BACK_RIGHT: {
                StartLocation = PoseInTiles(1.29, 0.4, West);
                drive.setPoseEstimate(StartLocation);
                threePurpleDropTrajectories.createThreePoseStart(new ThreePose(StartLocation));

                ThreePose purpleDropLocation = new ThreePose(
                        PoseInTiles(1.1, 1.99, North), //middle
                        PoseInTiles(1.52, 1.7, South), //left
                        PoseInTiles(1.383, 1.7, North) //right
                );

                Pose2d purpleHalfWayLocation = PoseInTiles(0.5,1,North);

                // Wait position trajectory //

                ThreePose waitLoc = new ThreePose(PoseInTiles(4.5, 2.15, North));

                opMode.telemetry.addData("purple right: ", purpleDropLocation.poseRight);
                opMode.telemetry.addData("purple Halfway: ", purpleHalfWayLocation);

                ThreePose between = new ThreePose(
                        purpleDropLocation.poseMiddle.plus(
                                PoseInTiles(0,2.1-purpleDropLocation.poseMiddle.getY()/TILE_LENGTH,0)),
                        purpleDropLocation.poseLeft.plus(
                                PoseInTiles(0,2.1-purpleDropLocation.poseLeft.getY()/TILE_LENGTH,0)),
                        purpleDropLocation.poseRight.plus(
                                PoseInTiles(-0.2,2.1-purpleDropLocation.poseRight.getY()/TILE_LENGTH,0))
                );

                threeMiddleForBackTrajectories.createThreePoseStart(purpleDropLocation);
                threeMiddleForBackTrajectories.addLineToSplineHeading(between);

                threeMiddleForBackTrajectories.addConstHeadingTraj(waitLoc);

                threeMiddleForBackTrajectories.endLocations = waitLoc;
                double correctPark = 3.5;
                threeYellowDropTrajectories.startLocations = threeMiddleForBackTrajectories.endLocations;
                threeMiddleForBackTrajectories.buildTrajs();

                //end of Wait Position trajectory //

                threePurpleDropTrajectories.trajbuilderRight
                        .lineToSplineHeading(purpleHalfWayLocation)
                        .splineToConstantHeading(purpleDropLocation.poseRight.vec(), purpleDropLocation.poseRight.getHeading());

                threePurpleDropTrajectories.trajbuilderLeft //add path to the left trajectory
                        .splineToLinearHeading(purpleDropLocation.poseLeft, purpleDropLocation.poseLeft.getHeading());

                threePurpleDropTrajectories.trajbuilderMiddle
                        .lineToSplineHeading(purpleHalfWayLocation)
                        .splineToConstantHeading(purpleDropLocation.poseMiddle.vec(), purpleDropLocation.poseMiddle.getHeading());
                threePurpleDropTrajectories.buildTrajs();
                break;
            }
            case FRONT_LEFT: {
                StartLocation = PoseInTiles(3.71, 5.6, East);
                drive.setPoseEstimate(StartLocation);
                threePurpleDropTrajectories.createThreePoseStart(new ThreePose(StartLocation));

                ThreePose purpleDropLocation = new ThreePose(
                        PoseInTiles(3.9, 4.01, North), //middle
                        PoseInTiles(3.48, 4.3, North), //left
                        PoseInTiles(3.617, 4.3, North) //right
                );

                Pose2d purpleHalfWayLocation = tryFront(tryRight(PoseInTiles(5.5,5,South)));

                threePurpleDropTrajectories.trajbuilderRight
                        .lineToSplineHeading(purpleHalfWayLocation)
                        .splineToConstantHeading(purpleDropLocation.poseRight.vec(), purpleDropLocation.poseRight.getHeading());

                threePurpleDropTrajectories.trajbuilderLeft //add path to the left trajectory
                        .splineToLinearHeading(purpleDropLocation.poseLeft, purpleDropLocation.poseLeft.getHeading());

                threePurpleDropTrajectories.trajbuilderMiddle
                        .lineToSplineHeading(purpleHalfWayLocation)
                        .splineToConstantHeading(purpleDropLocation.poseMiddle.vec(), purpleDropLocation.poseMiddle.getHeading());
                threePurpleDropTrajectories.buildTrajs();
                break;
            }
            case  FRONT_RIGHT: {
                StartLocation = PoseInTiles(3.71, 0.4, West);
                drive.setPoseEstimate(StartLocation);
                threePurpleDropTrajectories.createThreePoseStart(new ThreePose(StartLocation));

                ThreePose purpleDropLocation = new ThreePose(
                        PoseInTiles(3.9, 1.99, North), //middle
                        PoseInTiles(3.02, 1.7, North), //left
                        PoseInTiles(3.617, 1.7, North) //right
                );

                Pose2d purpleHalfWayLocation = tryFront(tryRight(PoseInTiles(5.5,1,South)));

                threePurpleDropTrajectories.trajbuilderLeft //add path to the left trajectory
                        .lineToSplineHeading(purpleHalfWayLocation)
                        .splineToConstantHeading(purpleDropLocation.poseLeft.vec(), purpleDropLocation.poseLeft.getHeading());

                threePurpleDropTrajectories.trajbuilderRight
                        .splineToLinearHeading(purpleDropLocation.poseRight, purpleDropLocation.poseRight.getHeading());

                threePurpleDropTrajectories.trajbuilderMiddle
                        .lineToSplineHeading(purpleHalfWayLocation)
                        .splineToConstantHeading(purpleDropLocation.poseMiddle.vec(), purpleDropLocation.poseMiddle.getHeading());
                threePurpleDropTrajectories.buildTrajs();
                break;
            }
        }
    }

    public int getXSignByPosition(){
        int sign = 1;
        if(!isBack()){
            sign = -1;
        }
        return sign;
    }

    public int getYSignByPosition(){
        int sign = 1;
        if(isRight()){
            sign = -1;
        }
        return sign;
    }

    public void runAfterInputTest(CameraSystem.DetectionLocation location){

        fourBarSystem.setMotorPower(0.85);
        propPlace = location;
        opMode.telemetry.addData("Detection ", propPlace);
        opMode.telemetry.update();

        //drive.followTrajectory(splineToPurple); //tod
        dropPurpleSeq.startSequence();

        threePurpleDropTrajectories.driveCorrectTrajectory(propPlace);

        // Drop Purple

        while (!opMode.isStopRequested()){
            if(dropPurpleSeq.isDone()){
                break;
            }
        }

        clawSystem.MoveOneClaw(true, true);
        opMode.sleep(1000);
        // Sequance drop
        returnFromPurple.startSequence();

        while (!opMode.isStopRequested()){
            if(returnFromPurple.isDone()){
                break;
            }
        }

    }
    public void runAfterInput() {

        fourBarSystem.setMotorPower(0.85);
        propPlace = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Detection ", propPlace);
        opMode.telemetry.update();

        //drive.followTrajectory(splineToPurple); //tod
        dropPurpleSeq.startSequence();

        threePurpleDropTrajectories.driveCorrectTrajectory(propPlace);

        // Drop Purple

        while (!opMode.isStopRequested()){
            if(dropPurpleSeq.isDone()){
                break;
            }
        }

        clawSystem.MoveOneClaw(true, true);
        opMode.sleep(1000);
        // Sequance drop
        returnFromPurple.startSequence();

        while (!opMode.isStopRequested()){
            if(returnFromPurple.isDone()){
                break;
            }
        }

    }

    public void SecondCall(){
        if(isBack()) {
            threeMiddleForBackTrajectories.driveCorrectTrajectory(propPlace);
            drive.setPoseEstimate(tryRight(PoseInTiles(4.5, 3.5, PI)));
            opMode.sleep(3000);
        }

        //both Back and Front//
        threeYellowDropTrajectories.driveCorrectTrajectory(propPlace);

        dropYellow.startSequence();

        while (!opMode.isStopRequested()) {
            if (dropYellow.isDone())
                break;
        }

        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .back(12.5).build());

        clawSystem.toggle();
        opMode.sleep(300);


        returnSystemsToStart.startSequence();
        threeParkTrajectories.driveCorrectTrajectory(propPlace);

        while (!opMode.isStopRequested()){
            if(returnSystemsToStart.isDone())
                break;
        }
    }
}