package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.TemperedTankDrive;

/*
 *   This is an example of a spline path for a tank drive robot, approximately following
 * a circular arc of 90 degrees.
 */
@Autonomous(name="Spline Arc", group = "drive")
public class SplineArc extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TemperedTankDrive drive = new TemperedTankDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        //   Move from the corner of one tile to the opposite corner, and rotating
        // 90 degrees CCW.
        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(24, 24), Math.toRadians(90))
                .build();

        drive.followTrajectory(traj);
    }
}
