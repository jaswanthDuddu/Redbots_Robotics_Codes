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

package org.firstinspires.ftc.teamcode.States2022;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Single Main Control", group="Jaswanth")


public class Jazzy_States_Teleop_Single_Main extends LinearOpMode {



    Jazzy_States_Hardware robot = new Jazzy_States_Hardware();   // Use a Redbot's hardware




    @Override
    public void runOpMode() {


        robot.init(hardwareMap);


        telemetry.addData("Yo Say", "\"Jaswanth is the freakin' GOAT!\" - Hasy");
        telemetry.update();


        waitForStart();


        while (opModeIsActive()) {



            Mecanum.Motion motion = Mecanum.joystickToMotion(
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x, -gamepad1.right_stick_y);

            // Convert desired motion to wheel powers, with power clamping
            Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
            robot.frontLeft.setPower((wheels.frontLeft)*0.82);
            robot.frontRight.setPower((wheels.frontRight)*0.82);
            robot.backLeft.setPower((wheels.backLeft)*0.82);
            robot.backRight.setPower((wheels.backRight)*0.82);



            if (!gamepad1.y && !gamepad1.a) {
                robot.lifter.setPower(0);
            }
            if (gamepad1.a) {
                robot.lifter.setPower(10);
            }
            if (gamepad1.y){
                robot.lifter.setPower(-10);
            }



            if (!gamepad1.x && !gamepad1.b) {
                robot.intake.setPower(0);
            }
            if (gamepad1.x) {
                robot.intake.setPower(1);
            }
            if (gamepad1.b) {
                robot.intake.setPower(-1);
            }
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                robot.duck.setPower(0);
            }
            if (gamepad1.left_bumper) {
                robot.duck.setPower(-0.475);
            }
            if (gamepad1.right_bumper) {
                robot.duck.setPower(0.475);
            }

            if(gamepad1.dpad_right)
            {
                robot.holder.setPosition(0.85);
            }
            if(gamepad1.dpad_left)
            {
                robot.holder.setPosition(0.4);
            }



        }
    }
}

