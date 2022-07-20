///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//
///**
// * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
// * All device access is managed through the HardwarePushbot class.
// * The code is structured as a LinearOpMode
// *
// * This particular OpMode executes a POV Game style Teleop for a PushBot
// * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
// * It raises and lowers the claw using the Gampad Y and A buttons respectively.
// * It also opens and closes the claws slowly using the left and right Bumper buttons.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@TeleOp(name="Ahen_Main_for_tour", group="Jaswanth")
//// @Disabled
//public class AhenTeleop1 extends LinearOpMode {
//
//    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
//
//    /* Declare OpMode members. */
//    HardwareRedbot robot = new HardwareRedbot();   // Use a Redbot's hardware
//
//    double target = 0;
//    double SERVO_SPEED = .1;
//    double yashIsaPanda = 0.008;
//    private ElapsedTime     runtime = new ElapsedTime();
//
//    @Override
//    public void runOpMode() {
//
//        /* Initialize the hardware variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
////        robot.detector.disable();
//        // Send telemetry, message to signify robot waiting;
//        telemetry.addData("Remember", "\"Testing the new robot.\" ");
//        telemetry.update();
//
//        // Which idiot wrote the documentation for this code? I have no idea what I'm doing - Ihba
//
//        //robot.autoServoLeft.setPosition(0.63);
//        //robot.autoServoRight.setPosition(0.45);
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//
//
//            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
//
//            // Convert joysticks to desired motion
//
//            Mecanum.Motion motion = Mecanum.joystickToMotion(
//                    gamepad1.left_stick_x, gamepad1.left_stick_y,
//                    gamepad1.right_stick_x, -gamepad1.right_stick_y);
//
//            // Convert desired motion to wheel powers, with power clamping
//            Mecanum.Wheels wheels = Mecanum.motionToWheels(motion);
//            robot.frontLeft.setPower(wheels.frontLeft*10);
//            robot.frontRight.setPower(wheels.frontRight*10);
//            robot.backLeft.setPower(wheels.backLeft*10);
//            robot.backRight.setPower(wheels.backRight*10);
//
//            //telemetry.addData("Motor speeds", "robot.frontLeft.getPower(), robot.frontRight.getPower(), robot.backLeft.getPower(), robot.backRight.getPower()", robot.backRight.getPower(), robot.backLeft.getPower(), robot.frontRight.getPower(), robot.frontLeft.getPower());
//            //telemetry.update();
//            if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
//                robot.intakeLeft.setPower(0);
//                robot.intakeRight.setPower(0);
//            }
//            if (gamepad2.dpad_right) {
//                robot.placer.setPosition(0.7);
//            }
//            if (gamepad2.dpad_left) {
//                robot.placer.setPosition(0);
//            }
//
//            if (gamepad2.left_bumper) {
//                robot.intakeLeft.setPower(0.4);
//                robot.intakeRight.setPower(-0.4);
//            }
//            if (gamepad2.right_bumper) {
//                robot.intakeLeft.setPower(-0.7);
//                robot.intakeRight.setPower(0.7);
//            }
//            if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
//                robot.slide.setPower(0);
//            }
//            if (gamepad2.dpad_down) {
//                robot.slide.setPower(-1);
//            }
//            if (gamepad2.dpad_up){
//                robot.slide.setPower(1);
//            }
//
//            if (gamepad2.a) {
//                robot.claw.setPosition(0);
//            }
//            if (gamepad2.b) {
//                robot.claw.setPosition(0.64);
//            }
//
//            if (gamepad2.y) {
//                robot.trayRight.setPosition(0.6);
//                robot.trayLeft.setPosition(0.6);
//            }
//            if (gamepad2.x) {
//                robot.trayRight.setPosition(0.28);
//                robot.trayLeft.setPosition(0.2);
//            }
//
//
////            if (digitalTouch.getState() == true) {
////                telemetry.addData("Digital Touch", "Is Not Pressed");
////                robot.intakeLeft.setPower(-1);
////                robot.intakeRight.setPower(1);
////            }
////            if ((digitalTouch.getState() == false)) {
////                robot.intakeLeft.setPower(0);
////                robot.intakeRight.setPower(0);
////                telemetry.addData("Digital Touch", "Is Pressed");
////            }
////            telemetry.update();
//
//
//        }
//    }
//}
//
