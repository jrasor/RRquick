package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

/*
 * This is an example of a simple spline path for a tank drive robot.
 *
 *   A spline is a movement path beginning with an arbitrary position and heading, and ending
 * with another arbitrary position and heading. For example, a robot could
 * o  start at 0, 0 and pointing along the x axis
 * o  move 24" forward and 24" to the right (-y axis)
 * o  ending with the same orientation, pointing along the x axis.
 *
 *    Traditionally, Teams running tank drive robots have done this by
 * o  turning 45 degrees CW
 * o  driving straight 34" (diagonal of a 24" square)
 * o  turning 45 degrees CCW.
 *
 *    Road Runner gives you the possibility of combining all this into a single smooth curving
 * motion:
 *
 *
 ==|==        ==|==
====================
||                ||
||                ||
||                ||o o o -->
||                ||      o o
||                ||          o
====================            o  \
 ==|==        ==|==              o  \           ==|==        ==|==
                                  o _\|        ====================
                                    o          ||                ||
                                      o o -->  ||                ||
                                          o o o||                ||
                                               ||                ||
                                               ||                ||
                                               ====================
                                                ==|==        ==|==
 */
@Autonomous(name="Simple Spline Test", group = "drive")
public class SplineSimple extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(24, -24), 0)
                .build();

        drive.followTrajectory(traj);
    }
}
