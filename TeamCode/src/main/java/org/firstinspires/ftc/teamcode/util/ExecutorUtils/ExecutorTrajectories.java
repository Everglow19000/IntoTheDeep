package org.firstinspires.ftc.teamcode.util.ExecutorUtils;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.EverglowLibrary.Systems.Executor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;

public class ExecutorTrajectories extends Executor {
    private final SampleMecanumDrive m_Drive;
    private final TrajectorySequence m_TrajectorySequence;
    private final Trajectory m_Trajectory;
    private final boolean m_IsSync;

    public ExecutorTrajectories(SampleMecanumDrive drive, TrajectorySequence trajectorySequence, boolean isSync){
        m_Drive = drive;
        m_TrajectorySequence = trajectorySequence;
        m_Trajectory = null;
        m_IsSync = isSync;
    }

    public ExecutorTrajectories(SampleMecanumDrive drive, Trajectory trajectory, boolean isSync){
        m_Drive = drive;
        m_TrajectorySequence = null;
        m_Trajectory = trajectory;
        m_IsSync = isSync;
    }

    @Override
    public boolean isFinished() {
        return !m_Drive.isBusy();
    }

    @Override
    public void stop() {

    }

    @Override
    public void run() {
        if(m_Trajectory == null){
            if(m_IsSync)
                m_Drive.followTrajectorySequence(m_TrajectorySequence);
            else
                m_Drive.followTrajectorySequenceAsync(m_TrajectorySequence);
        }
        else{
            if(m_IsSync)
                m_Drive.followTrajectory(m_Trajectory);
            else
                m_Drive.followTrajectoryAsync(m_Trajectory);
        }
    }
}
