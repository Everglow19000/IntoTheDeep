package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import com.acmerobotics.roadrunner.geometry.Pose2d;

public class ThreePose {
    public Pose2d poseMiddle, poseLeft, poseRight;
    ThreePose(Pose2d poseMiddle, Pose2d poseLeft, Pose2d poseRight) {
        this.poseMiddle = poseMiddle;
        this.poseLeft = poseLeft;
        this.poseRight = poseRight;
    }
    public ThreePose(Pose2d poseAll) {
        this.poseMiddle = poseAll;
        this.poseLeft = poseAll;
        this.poseRight = poseAll;
    }

    ThreePose() {
        this.poseMiddle = new Pose2d();
        this.poseLeft = new Pose2d();
        this.poseRight = new Pose2d();
    }

    public ThreePose plus(ThreePose other){
        return new ThreePose(
                this.poseMiddle.plus(other.poseMiddle),
                this.poseLeft.plus(other.poseLeft),
                this.poseRight.plus(other.poseRight)
        );
    }
}