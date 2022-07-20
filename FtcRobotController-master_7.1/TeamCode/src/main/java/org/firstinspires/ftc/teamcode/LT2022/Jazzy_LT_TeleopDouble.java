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
//@TeleOp(name="HSAY_Double", group="Jaswanth")
//// @Disabled
//public class Jazzy_LT_TeleopDouble extends LinearOpMode {
//
//    final double    CLAW_SPEED      = 0.02 ;                   // sets rate to move servo
//
//    /* Declare OpMode members. */
//    Jazzy_LT_Hardware robot = new Jazzy_LT_Hardware();   // Use a Redbot's hardware
//
//    double target = 0;
//    double SERVO_SPEED = .1;
//    double yashIsaPanda = 0.008;
//
//    boolean armOut = false;
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
//        telemetry.addData("Yo ", "\"Testing the new robot.\" ");
//        telemetry.update();
//        telemetry.addData("Say", "\"Jaswanth is the freakin' GOAT!\" - Hasy");
//        telemetry.update();
//
//        // Which idiot wrote the documentation for this code? I have no idea what I'm doing - Ihba
//
//        //robot.autoServoLeft.setPosition(0.63);
//        //robot.autoServoRight.setPosition(0.45);
//        // Wait for the game to start (driver presses PLAY)
//        //robot.turner.setPosition(0.25);
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
//            robot.frontLeft.setPower((wheels.frontLeft)*0.82);
//            robot.frontRight.setPower((wheels.frontRight)*0.82);
//            robot.backLeft.setPower((wheels.backLeft)*0.82);
//            robot.backRight.setPower((wheels.backRight)*0.82);
//
//
//            if(gamepad1.x)
//            {
//                robot.capArm.setPosition(0.1);
//            }
//            if(gamepad1.y)
//            {
//                robot.capArm.setPosition(0.5);
//            }
//
////            if (!gamepad1.dpad_up && !gamepad1.dpad_down) {
////                robot.capHand.setPower(0);
////            }
//            if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad2.a && !gamepad2.b) {
//                robot.capHand.setPower(0);
//            }
//            if (gamepad1.dpad_down) {
//                robot.capHand.setPower(-0.5);
//            }
//            if (gamepad1.dpad_up){
//                robot.capHand.setPower(0.5);
//            }
//            if (gamepad2.a) {
//                robot.capHand.setPower(-0.5);
//            }
//            if (gamepad2.b){
//                robot.capHand.setPower(0.5);
//            }
//
//            if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
//                robot.lifter.setPower(0);
//            }
//            if (gamepad2.dpad_down) {
//                robot.lifter.setPower(1);
//            }
//            if (gamepad2.dpad_up){
//                robot.lifter.setPower(-1);
//            }
//
//
//
//            if (!gamepad2.dpad_left && !gamepad2.dpad_right) {
//                robot.intake.setPower(0);
//            }
//            if (gamepad2.dpad_right) {
//                robot.intake.setPower(1);
//            }
//            if (gamepad2.dpad_left) {
//                robot.intake.setPower(-1);
//            }
//            if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
//                robot.duck.setPower(0);
//            }
//            if (gamepad2.left_bumper) {
//                robot.duck.setPower(-0.45);
//            }
//            if (gamepad2.right_bumper) {
//                robot.duck.setPower(0.45);
//            }
//
//            if(gamepad2.x)
//            {
//                robot.holder.setPosition(0.85);
//            }
//            if(gamepad2.y)
//            {
//                robot.holder.setPosition(0.4);
//            }
//
////            if(!gamepad2.a && !gamepad2.b)
////            {
////                robot.lifter.setPower(0);
////            }
//
////            if(gamepad2.a && armOut == true)
////            {
////                encoderArm(0.712);
////                armOut = false;
////            }
////            if(gamepad2.b && armOut == false)
////            {
////                encoderArm(-0.712);
////                armOut = true;
////            }
//
//
//
//
//
//
//
//        }
//    }
////    public void encoderArm(double inches)
////    {
////        double speed = 1;
////        int newTarget;
////        if (opModeIsActive()) {
////            newTarget = robot.lifter.getCurrentPosition() + (int)((inches*10) * (1120 / (0.75 * 3.1415)));
////            robot.lifter.setTargetPosition(newTarget);
////            double inch;
////
////            //hsaY
////
////            //encoderDrive(0.2,0,0,3);
////            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////
////            robot.lifter.setPower(Math.abs(speed*10));
////            while (opModeIsActive() && (robot.lifter.isBusy())) {
////                // Display it for the driver.
////                telemetry.addData("Path1",  "Running to %7d :", newTarget);
////                telemetry.addData("Path2",  "Running at %7d :",
////                        robot.lifter.getCurrentPosition());
////                telemetry.update();
////            }
////
////            // Stop all motion;
////
////            robot.lifter.setPower(0);
////
////            // Turn off RUN_TO_POSITION
////            robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////
////            //sleep(250);   // optional pause after each move
////        }
////    }
//
//}
//
