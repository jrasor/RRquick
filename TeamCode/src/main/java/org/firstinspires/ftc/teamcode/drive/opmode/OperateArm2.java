/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.round;

/**
 *      This OpMode scans a single servo back and forwards until Stop is pressed, following
 * a sigmoid angle vs time function over the interval (0, 1):
 *
 *      r (t) = 0.5 - 0.5 * cos (πt)
 *
 *      This function starts slowly from zero, builds up some speed until the halfway
 *  point is reached, then slows down to gently approach the endpoint.
 *      The domain and range can be scaled.
 *
 * The code is structured as a LinearOpMode.
 * CYCLE_MS sets the update period.
 *
 * This code assumes two Servos configured with the name "servo0" and "servo1" as found
 * on a Trainerbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure
 * that any other connected servos are able to move freely before running this test.
 */

@TeleOp(name = "Operate Arm Gently", group = "Actuators")
//@Disabled
public class OperateArm2 extends LinearOpMode {
    static final int    CYCLE_MS    =   50;     // period of each report cycle
    // The two servos are mounted facing each other, so forward on one is reverse
    // on the other.
    static final double LEFT_STOWED      =  1.0;     // Retracted over robot body
    static final double LEFT_DEPLOYED    =  0.0;     // Extended out over Field
    static final double RIGHT_STOWED      =  0.0;     // Retracted over robot body
    static final double RIGHT_DEPLOYED    =  1.0;     // Extended out over Field
 //   static final double HALFWAY     =  (STOWED - DEPLOYED)/2;
    double time                     = 0.0;        // Time since test began.
    // At scale 1.0, 1.0 second between extreme positions STOWED and DEPLOYED.
    double timeScale                     = 0.3;

    // Define class members
    Servo leftServo;
    Servo rightServo;
    double leftPosition = LEFT_STOWED;
    double rightPosition = RIGHT_STOWED;

    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();

        // Change the text in quotes to match any servo name on your robot.
        leftServo = hardwareMap.get(Servo.class, "servo0");
        rightServo = hardwareMap.get(Servo.class, "servo1");

        // Wait for the start button
        telemetry.addData(">", "Press Start to activate arm." );
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Scan servo till stop pressed.
        while(opModeIsActive()){
            // Move servos to current position
            //leftServo.setPosition(leftPosition);
            rightServo.setPosition(rightPosition);

            //  Report on time and position
            telemetry.addData("Run time", "%5.3f", time);
            telemetry.addData("Left servo position", "%5.2f", leftPosition);
            telemetry.addData("Right servo position", "%5.2f", rightPosition);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // update time and positions
            time = runtime.time();
            leftPosition = 0.5 - 0.5 * Math.cos(timeScale * Math.PI * time);
            rightPosition = 0.5 + 0.5 * Math.cos(timeScale * Math.PI * time);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
