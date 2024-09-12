package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ThreeTrajectories {
    ThreePose startLocations = new ThreePose(), middleLocations = new ThreePose(),  endLocations = new ThreePose();
    public Trajectory trajMiddle, trajLeft, trajRight;

    public TrajectoryBuilder trajbuilderMiddle, trajbuilderLeft, trajbuilderRight;
    private final SampleMecanumDrive drive;


    public void setStartPose(Pose2d startLocation) {
        startLocations = new ThreePose(startLocation, startLocation, startLocation);
    }

    public ThreeTrajectories(SampleMecanumDrive drive){
        this.drive = drive;
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

    public void addSpline(ThreePose poses) {
        trajbuilderMiddle.splineTo(poses.poseMiddle.vec(), poses.poseMiddle.getHeading());
        trajbuilderLeft.splineTo(poses.poseLeft.vec(), poses.poseLeft.getHeading());
        trajbuilderRight.splineTo(poses.poseRight.vec(), poses.poseRight.getHeading());
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

    public void createLinerHeadingTrajectories(boolean reversed) {
        trajMiddle = trajToLinearHeading(startLocations.poseMiddle, endLocations.poseMiddle, reversed);
        trajLeft = trajToLinearHeading(startLocations.poseLeft, endLocations.poseLeft, reversed);
        trajRight = trajToLinearHeading(startLocations.poseRight, endLocations.poseRight, reversed);
    }

    public void createConstHeadingTrajectories(boolean reversed) {
        trajMiddle = trajToConstantHeading(startLocations.poseMiddle, endLocations.poseMiddle, reversed);
        trajLeft = trajToConstantHeading(startLocations.poseLeft, endLocations.poseLeft, reversed);
        trajRight = trajToConstantHeading(startLocations.poseRight, endLocations.poseRight, reversed);
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

    private Trajectory trajToPose(Pose2d poseStart, Pose2d poseFinish) {
        return drive.trajectoryBuilder(poseStart)
                .splineTo(poseFinish.vec(), poseFinish.getHeading())
                .build();
    }
    private Trajectory trajToConstantHeading(Pose2d startPose, Pose2d endPose, boolean reversed){
        return drive.trajectoryBuilder(startPose, reversed)
                .splineToConstantHeading(endPose.vec(),endPose.getHeading())
                .build();
    }
    private Trajectory trajToLinearHeading(Pose2d startPose, Pose2d endPose, boolean isReversed){
        return drive.trajectoryBuilder(startPose, isReversed)
                .splineToLinearHeading(endPose,endPose.getHeading())
                .build();
    }

    private Trajectory trajMiddlePose(Pose2d poseStart,  Pose2d poseMiddle, Pose2d poseFinish) {
        return drive.trajectoryBuilder(poseStart)
                .splineTo(poseMiddle.vec(), poseMiddle.getHeading())
                .splineTo(poseFinish.vec(), poseFinish.getHeading())
                .build();
    }
    private Trajectory trajMiddleConstantHeading(Pose2d poseStart,  Pose2d poseMiddle, Pose2d poseFinish){
        return drive.trajectoryBuilder(poseStart)
                .splineToConstantHeading(poseMiddle.vec(),poseMiddle.getHeading())
                .splineToConstantHeading(poseFinish.vec(),poseFinish.getHeading())
                .build();
    }
    private Trajectory trajMiddleLinearHeading(Pose2d poseStart, Pose2d poseMiddle, Pose2d poseFinish){
        return drive.trajectoryBuilder(poseStart)
                .splineToLinearHeading(poseMiddle, poseMiddle.getHeading())
                .splineToLinearHeading(poseFinish, poseFinish.getHeading())
                .build();
    }

    private Trajectory trajLinetoSplineHeading(Pose2d poseStart, Pose2d poseFinish){
        return drive.trajectoryBuilder(poseStart)
                .lineToSplineHeading(poseFinish)
                .build();
    }

    public void driveCorrectTrajectory(CameraSystem.DetectionLocation propPlace) {
        switch (propPlace) {
            case MIDDLE:
                drive.followTrajectory(trajMiddle);
                break;
            case LEFT:
                drive.followTrajectory(trajLeft);
                break;
            case RIGHT:
                drive.followTrajectory(trajRight);
                break;
        }
    }

    public void createThreePoseStart(ThreePose threePose, boolean reversed){
        trajbuilderMiddle = drive.trajectoryBuilder(threePose.poseMiddle, reversed);
        trajbuilderLeft = drive.trajectoryBuilder(threePose.poseLeft, reversed);
        trajbuilderRight = drive.trajectoryBuilder(threePose.poseRight, reversed);
    }
    public void createThreePoseStart(ThreePose threePose, double angle){
        trajbuilderMiddle = drive.trajectoryBuilder(threePose.poseMiddle, angle);
        trajbuilderLeft = drive.trajectoryBuilder(threePose.poseLeft, angle);
        trajbuilderRight = drive.trajectoryBuilder(threePose.poseRight, angle);
    }
    public void createThreePoseStart(ThreePose threePose){
        trajbuilderMiddle = drive.trajectoryBuilder(threePose.poseMiddle);
        trajbuilderLeft = drive.trajectoryBuilder(threePose.poseLeft);
        trajbuilderRight = drive.trajectoryBuilder(threePose.poseRight);
    }

    public void createLineToSplineHeading(){
        trajMiddle = trajLinetoSplineHeading(startLocations.poseMiddle, endLocations.poseMiddle);
        trajLeft = trajLinetoSplineHeading(startLocations.poseLeft, endLocations.poseLeft);
        trajRight = trajLinetoSplineHeading(startLocations.poseRight, endLocations.poseRight);
    }

    public void addForward(double distance){
        if(distance < 0){
            distance = -distance;
            trajbuilderMiddle.back(distance);
            trajbuilderLeft.back(distance);
            trajbuilderRight.back(distance);
        }
        else{
            trajbuilderMiddle.forward(distance);
            trajbuilderLeft.forward(distance);
            trajbuilderRight.forward(distance);
        }
    }
}
