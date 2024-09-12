package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.EverglowLibrary.Systems.CameraSystem;
import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.EverglowLibrary.ThreadHandleLib.SequenceRunner;
import org.EverglowLibrary.utils.PointD;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


public class FourtyFivePoints {

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
    public Pose2d PoseInTiles(double x, double y, double Heading) {
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

    public Trajectory trajToPose(Pose2d poseStart, Pose2d poseFinish) {
        return drive.trajectoryBuilder(poseStart)
                .splineTo(poseFinish.vec(), poseFinish.getHeading())
                .build();
    }
    public Trajectory trajToConstantHeading(Pose2d startPose, Pose2d endPose){
        return drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(endPose.vec(),endPose.getHeading())
                .build();
    }
    public Trajectory trajToLinearHeading(Pose2d startPose, Pose2d endPose){
        return drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(endPose,endPose.getHeading())
                .build();
    }

    public Trajectory trajMiddlePose(Pose2d poseStart,  Pose2d poseMiddle, Pose2d poseFinish) {
        return drive.trajectoryBuilder(poseStart)
                .splineTo(poseMiddle.vec(), poseMiddle.getHeading())
                .splineTo(poseFinish.vec(), poseFinish.getHeading())
                .build();
    }
    public Trajectory trajMiddleConstantHeading(Pose2d poseStart,  Pose2d poseMiddle, Pose2d poseFinish){
        return drive.trajectoryBuilder(poseStart)
                .splineToConstantHeading(poseMiddle.vec(),poseMiddle.getHeading())
                .splineToConstantHeading(poseFinish.vec(),poseFinish.getHeading())
                .build();
    }
    public Trajectory trajMiddleLinearHeading(Pose2d poseStart, Pose2d poseMiddle, Pose2d poseFinish){
        return drive.trajectoryBuilder(poseStart)
                .splineToLinearHeading(poseMiddle, poseMiddle.getHeading())
                .splineToLinearHeading(poseFinish, poseFinish.getHeading())
                .build();
    }

    public class ThreeTrajectories {
        public ThreePose startLocations = new ThreePose(), middleLocations = new ThreePose(),  endLocations = new ThreePose();
        public Trajectory trajMiddle, trajLeft, trajRight;

        public TrajectoryBuilder trajbuilderMiddle, trajbuilderLeft, trajbuilderRight;


        public void setStartPose(Pose2d startLocation) {
            startLocations = new ThreePose(startLocation, startLocation, startLocation);
        }

        public void setMiddlePose(Pose2d startLocation) {
            startLocations = new ThreePose(startLocation, startLocation, startLocation);
        }
        public void setEndPose(Pose2d endLocation) {
            endLocations = new ThreePose(endLocation, endLocation, endLocation);
        }

        public void addConstHeadingTraj(ThreePose poses) {
            trajbuilderMiddle.splineToConstantHeading(poses.poseMiddle.vec(), poses.poseMiddle.getHeading());
            trajbuilderLeft.splineToConstantHeading(poses.poseLeft.vec(), poses.poseLeft.getHeading());
            trajbuilderRight.splineToConstantHeading(poses.poseRight.vec(), poses.poseRight.getHeading());
        }

        public void addLinearHeadingTraj(ThreePose poses) {
            trajbuilderMiddle.splineToLinearHeading(poses.poseMiddle, poses.poseMiddle.getHeading());
            trajbuilderLeft.splineToLinearHeading(poses.poseLeft, poses.poseLeft.getHeading());
            trajbuilderRight.splineToLinearHeading(poses.poseRight, poses.poseRight.getHeading());
        }

        public void addLineToSplineHeading(ThreePose poses) {
            trajbuilderMiddle.lineToSplineHeading(poses.poseMiddle);
            trajbuilderLeft.lineToSplineHeading(poses.poseLeft);
            trajbuilderRight.lineToSplineHeading(poses.poseRight);
        }

        public void createTrajectories() {
            trajMiddle = trajToPose(startLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajToPose(startLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajToPose(startLocations.poseRight, endLocations.poseRight);
        }

        public void buildTrajs(){
            trajMiddle = trajbuilderMiddle.build();
            trajLeft = trajbuilderLeft.build();
            trajRight = trajbuilderRight.build();
        }

        public void createLinerHeadingTrajectories() {
            trajMiddle = trajToLinearHeading(startLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajToLinearHeading(startLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajToLinearHeading(startLocations.poseRight, endLocations.poseRight);
        }

        public void createConstHeadingTrajectories() {
            trajMiddle = trajToConstantHeading(startLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajToConstantHeading(startLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajToConstantHeading(startLocations.poseRight, endLocations.poseRight);
        }

        public void createMiddleTrajectories() {
            trajMiddle = trajMiddlePose(startLocations.poseMiddle, middleLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajMiddlePose(startLocations.poseLeft, middleLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajMiddlePose(startLocations.poseRight, middleLocations.poseRight, endLocations.poseRight);
        }

        public void createMiddleLinerHeadingTrajectories() {
            trajMiddle = trajMiddleLinearHeading(startLocations.poseMiddle, middleLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajMiddleLinearHeading(startLocations.poseLeft, middleLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajMiddleLinearHeading(startLocations.poseRight, middleLocations.poseRight, endLocations.poseRight);
        }

        public void createMiddleConstHeadingTrajectories() {
            trajMiddle = trajMiddleConstantHeading(startLocations.poseMiddle, middleLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajMiddleConstantHeading(startLocations.poseLeft, middleLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajMiddleConstantHeading(startLocations.poseRight, middleLocations.poseRight, endLocations.poseRight);
        }


        public void driveCorrectTrajectory() {
            switch (propPlace) {
                case MIDDLE:
                    drive.followTrajectory(trajMiddle);
                case LEFT:
                    drive.followTrajectory(trajLeft);
                case RIGHT:
                    drive.followTrajectory(trajRight);
            }
        }

        public void createThreePoseStart(ThreePose threePose){
            trajbuilderMiddle = drive.trajectoryBuilder(threePose.poseMiddle);
            trajbuilderRight = drive.trajectoryBuilder(threePose.poseRight);
            trajbuilderLeft = drive.trajectoryBuilder(threePose.poseLeft);
        }
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
    static double distanceBetweenTags=15.0; //in cm
    static double distanceBuffer=0;

    static final double North = 0;
    static final double East = -PI/2;
    static final double West = PI/2;
    static final double South = PI;

    static final double TILE_LENGTH = 60.5;


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
    ThreeTrajectories threePurpleDropTrajectories = new ThreeTrajectories();

    ThreeTrajectories threeYellowDropTrajectories = new ThreeTrajectories();
    ThreeTrajectories threeParkTrajectories = new ThreeTrajectories();
    ThreeTrajectories threeMiddleForBackTrajectories = new ThreeTrajectories();

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
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cameraSystem = new CameraSystem(opMode, isRight(), isBack());


        elevatorSystem = new ElevatorSystem(opMode);
        clawSystem = new ClawSystem(opMode);
        fourBarSystem = new FourBarSystem(opMode);
        gWheelSystem = new GWheelSystem(opMode);

        fourBarSystem.restart();

        sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        sequenceRunner = new SequenceRunner();



        // Create all Sequences //
        dropPurpleSeq  = new Sequence(false,
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.LOW, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)
        );

        dropYellow = new Sequence(false, clawSystem.getExecutor(false)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)
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
        /*if(isRight()) {
            startLocation.minus(PoseInTiles(0.27, 0, 0));
        }*/
        StartLocation = tryRight(StartLocation);
        drive.setPoseEstimate(StartLocation);


        // Simple drive to Purple //

        /*Pose2d middleDropLocation = PoseInTiles(3.55, 4.5, East);
        if(isBack()) {
            opMode.telemetry.addLine("move the pose...");
            middleDropLocation = middleDropLocation.minus(PoseInTiles(2.1, 0, 0));
        }
        splineToPurple = trajToConstantHeading(tryRight(StartLocation), tryRight(middleDropLocation));

        if(isBack()) {

            //threeMiddleForBackTrajectories.setStartPose(tryRight(new Pose2d(middleDropLocation.getX(), middleDropLocation.getY(), South)));

            Pose2d waitLocation = PoseInTiles(4.5, 3.5, South), waitMiddleLocation = PoseInTiles(1.5, 3.5, South);
            *//*threeMiddleForBackTrajectories.setMiddlePose(tryRight(waitMiddleLocation));
            threeMiddleForBackTrajectories.setEndPose(tryRight(waitLocation));
            threeMiddleForBackTrajectories.createMiddleTrajectories();*//*

            trajMiddleForBack = trajMiddleLinearHeading(tryRight(new Pose2d(middleDropLocation.getX(), middleDropLocation.getY(), South)),
                    tryRight(waitMiddleLocation),
                    tryRight(waitLocation));

            threeYellowDropTrajectories.setStartPose((tryRight(waitLocation)));
        }

        else{
            threeYellowDropTrajectories.setStartPose(tryRight(new Pose2d(middleDropLocation.getX(), middleDropLocation.getY(), South)));
        }*/

        // Complex drive to Purple //

        threePurpleDropTrajectories.createThreePoseStart(new ThreePose(StartLocation));
        //threePurpleDropTrajectories.setStartPose(tryRight(StartLocation));

        if(isBack()) {
            ThreePose purpleDropLocation = new ThreePose(
                    tryRight(PoseInTiles(1.6, 4.153, South)),
                    tryRight(PoseInTiles(1.1, 4.5, South)),
                    tryRight(PoseInTiles(0.659, 4.153, South))
            );

            Pose2d purpleHalfWayLocation = tryRight(PoseInTiles(0.5,5,East));

            threePurpleDropTrajectories.addConstHeadingTraj(new ThreePose(purpleHalfWayLocation));
            threePurpleDropTrajectories.addLinearHeadingTraj(purpleDropLocation);
            threePurpleDropTrajectories.buildTrajs();
            //threePurpleDropTrajectories.endLocations.poseMiddle = tryRight(PoseInTiles(1.6, 4.153, South));
            //threePurpleDropTrajectories.endLocations.poseLeft = tryRight(PoseInTiles(1.1, 4.5, East));
            //threePurpleDropTrajectories.endLocations.poseRight = tryRight(PoseInTiles(0.659, 4.153, South)); //x: 0.659, y: 4.328
            //threePurpleDropTrajectories.createLinerHeadingTrajectories();



            Pose2d waitPosition = tryRight(PoseInTiles(3.5, 3.5, South));
            Pose2d middleWayToWaitPosition = tryRight(PoseInTiles(2, 3.5, South));
            threeMiddleForBackTrajectories.startLocations = threePurpleDropTrajectories.endLocations;

            threeMiddleForBackTrajectories.trajMiddle = drive.trajectoryBuilder(threeMiddleForBackTrajectories.startLocations.poseMiddle)
                    .splineToConstantHeading(middleWayToWaitPosition.vec(), middleWayToWaitPosition.getHeading())
                    .splineToConstantHeading(waitPosition.vec(), waitPosition.getHeading())
                    .build();
            threeMiddleForBackTrajectories.trajLeft = drive.trajectoryBuilder(threeMiddleForBackTrajectories.startLocations.poseLeft)
                    .splineToConstantHeading(middleWayToWaitPosition.vec(), middleWayToWaitPosition.getHeading())
                    .splineToConstantHeading(waitPosition.vec(), waitPosition.getHeading())
                    .build();
            threeMiddleForBackTrajectories.trajRight = drive.trajectoryBuilder(threeMiddleForBackTrajectories.startLocations.poseRight)
                    .splineToConstantHeading(middleWayToWaitPosition.vec(), middleWayToWaitPosition.getHeading())
                    .splineToConstantHeading(waitPosition.vec(), waitPosition.getHeading())
                    .build();



            threeYellowDropTrajectories.startLocations = threeMiddleForBackTrajectories.endLocations;
        }

        else{
            threePurpleDropTrajectories.endLocations.poseMiddle = tryRight(PoseInTiles(4.2, 4, East));
            threePurpleDropTrajectories.endLocations.poseLeft = tryRight(PoseInTiles(4.35, 3.5, East));
            threePurpleDropTrajectories.endLocations.poseRight =tryRight( PoseInTiles(0.39, 3.5, East));
            threePurpleDropTrajectories.createConstHeadingTrajectories();


            threeYellowDropTrajectories.startLocations = threePurpleDropTrajectories.endLocations;
        }

        // Yellow Traj


        threeYellowDropTrajectories.endLocations.poseMiddle = tryRight(PoseInTiles(5, 4.5, South));
        threeYellowDropTrajectories.endLocations.poseLeft =
                threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                        new Pose2d(0, distanceBetweenTags, 0));
        threeYellowDropTrajectories.endLocations.poseRight
                = threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                new Pose2d(0, -distanceBetweenTags, 0));

        threeYellowDropTrajectories.createConstHeadingTrajectories();



        // Park Traj //

        Pose2d parkLocation = PoseInTiles(5.4, 5.6, South), parkMiddleLocation = PoseInTiles(4.8, 5.3, South);
        if(isBack()) {
            parkLocation = parkLocation.minus(PoseInTiles(0, 2, 0));
            parkMiddleLocation = parkMiddleLocation.minus(PoseInTiles(0, 1.2, 0));
        }

        threeParkTrajectories.setEndPose(tryRight(parkLocation));
        threeParkTrajectories.setMiddlePose(tryRight(parkMiddleLocation));
        threeParkTrajectories.startLocations = threeYellowDropTrajectories.endLocations;
        threeParkTrajectories.createMiddleTrajectories();

        clawSystem.MoveOneClaw(true);

        opMode.telemetry.addLine("Ready!!!!! ");
        opMode.telemetry.update();

        opMode.waitForStart();

        runAfterInput();

    }
    
    
    public void runAfterInput() {

        fourBarSystem.setMotorPower(0.85);
        propPlace = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Detection ", propPlace);
        opMode.telemetry.update();

        //drive.followTrajectory(splineToPurple); //tod
        threePurpleDropTrajectories.driveCorrectTrajectory();

        /*opMode.telemetry.addData("Location ", LocationInTiles());
        opMode.telemetry.addData("start ", threeYellowDropTrajectories.startLocations.poseMiddle);
        opMode.telemetry.update();*/


        // Drop Purple
        dropPurpleSeq.startSequence();

        while (!opMode.isStopRequested()){
            if(dropPurpleSeq.isDone()){
                break;
            }
        }

        switch (propPlace) {
            case LEFT:
                drive.turn(-PI/2 * 1.4);
                break;
            case RIGHT:
                drive.turn(PI/2 * 1.4);
                break;
            case MIDDLE:
                drive.turn(PI);
                break;
        }
        clawSystem.ChangePos(true);
        opMode.sleep(1000);
        // Sequance drop
        returnFromPurple.startSequence();

        /*double devation = trueAngle(PI - LocationInTiles().getHeading());
        while(devation > toRadians(1)) {
            drive.setWeightedDrivePower(new Pose2d(0, 0, devation * abs(devation) * 0.01));
            devation = trueAngle(PI - LocationInTiles().getHeading());
            opMode.telemetry.addData("devation", devation);
            opMode.telemetry.update();
        }*/

        //drive.setWeightedDrivePower(new Pose2d(0, 0, 0));

        //turnTo(South);
        //sleep(1000);
        //turnTo(South);

        while (!opMode.isStopRequested()){
            if(returnFromPurple.isDone()){
                break;
            }
        }
        /*
        switch (propPlace) {
            case LEFT:
                drive.turn(PI/2 * 1.4);
                break;
            case RIGHT:
                drive.turn(-PI/2 * 1.4);
                break;
            case MIDDLE:
                drive.turn(-PI);
                break;
        }

         */

        if(!isBack()) {
            if(!isRight()) {
                tempPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(StartLocation.vec(), 0)
                        .splineTo(PoseInTiles(5.5, 5.5, 0).vec(), 0)
                        .build();
            }
            else{
                tempPark = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineTo(StartLocation.vec(), 0)
                        .splineTo(PoseInTiles(5.5, 0.5, 0).vec(), 0)
                        .build();
            }

            drive.followTrajectory(tempPark);

            gWheelSystem.setPower(1);

            opMode.sleep(2500);

            gWheelSystem.setPower(0);
        }
        /*
        if(isBack()) {
            drive.followTrajectory(trajMiddleForBack);
            //threeMiddleForBackTrajectories.driveCorrectTrajectory();
            opMode.sleep(10 * 1000);
        }

        dropYellow.startSequence();


        //*opMode.telemetry.addData("end ", threeYellowDropTrajectories.endLocations.poseMiddle);
        //opMode.telemetry.update();

        threeYellowDropTrajectories.driveCorrectTrajectory();

        while (!opMode.isStopRequested()){
            if(dropYellow.isDone()){
                clawSystem.ChangePos(true);
                sleep(600);
                break;
            }
        }

        returnSystemsToStart.startSequence();

        while (!opMode.isStopRequested()){
            if(returnSystemsToStart.isDone()){
                break;
            }
        }

        //threeParkTrajectories.driveCorrectTrajectory();

         */

    }

    public void SecondCall(){
        Trajectory backWaitForMove;
        Trajectory betweenWait = null;
        if(isBack()) {
            if (propPlace != CameraSystem.DetectionLocation.MIDDLE) {
                if (isRight()) {
                    backWaitForMove = drive.trajectoryBuilder(PoseInTiles(1.5,2.5,0))
                            .lineToConstantHeading(PoseInTiles(4.5, 2.5, 0).vec())
                            .build();
                    betweenWait = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(PoseInTiles(1.5, 2.5, PI))
                            .build();
                } else {
                    backWaitForMove = drive.trajectoryBuilder(PoseInTiles(1.5, 3.5,PI))
                            .lineToConstantHeading(PoseInTiles(4.5, 3.5, 0).vec())
                            .build();
                    betweenWait = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(PoseInTiles(1.5, 3.5, PI))
                            .build();
                }
            }
            else {
                if(isRight()){
                    backWaitForMove = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(PoseInTiles(0.5, 4.5, PI))
                            .splineTo(PoseInTiles(2.5, 2.5, 0).vec(), -PI)
                            .splineToConstantHeading(PoseInTiles(4.5, 2.5, 0).vec(), 0)
                            .build();
                }
                else {
                    backWaitForMove = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(PoseInTiles(0.5, 4.5, PI))
                            .splineTo(PoseInTiles(0.5, 3.5, 0).vec(), PI)
                            .splineToConstantHeading(PoseInTiles(4.5, 3.5, 0).vec(), 0)
                            .build();
                }
            }

            if (betweenWait != null) {
                drive.followTrajectory(betweenWait);
            }
            drive.followTrajectory(backWaitForMove);
            //opMode.sleep(10000);
            threeYellowDropTrajectories.driveCorrectTrajectory();
            //drop
            //gWheelSystem.setPower(1);

            //opMode.sleep(2500);

            //gWheelSystem.setPower(0);
        }
    }

}