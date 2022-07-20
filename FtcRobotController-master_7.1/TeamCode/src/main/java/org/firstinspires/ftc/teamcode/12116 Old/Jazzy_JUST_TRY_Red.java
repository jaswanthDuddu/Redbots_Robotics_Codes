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
//import android.graphics.Color;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//@Autonomous(name="HSAY_Red", group="Jaswanth")
//public class Jazzy_JUST_TRY_Red extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    JazzyHardware         robot   = new JazzyHardware();   // Use a Pushbot's hardware
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device\
//    private ElapsedTime     runtime = new ElapsedTime();
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    // These constants define the desired driving/control characteristics
//    // The can/should be tweaked to suite the specific robot drive train.
//    static final double     SLOW_DRIVE_SPEED        = 0.3;
//    static final double     DRIVE_SPEED             = 0.4;
//    static final double     TURN_SPEED              = 0.3;
//    static final double     LIFT_SPEED              = 0.5;
//    static final double     STRAFE_SPEED            = 0.3;
//
//    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
//    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
//    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
//
//    double currentAngle = 0;
//
//    static final double SERVO_DEPLOYED = 0.5;
//    static final double SERVO_RETRACTED = 0;
//    float hsvValues[] = {0F, 0F, 0F};
//    float hsvValues2[] = {0F, 0F, 0F};
//
//    // values is a reference to the hsvValues array.
//    final float values[] = hsvValues;
//    final float values2[] = hsvValues2;
//
//    final double SCALE_FACTOR = 255;
//
//    @Override
//    public void runOpMode() {
//        robot.init(hardwareMap);
//        telemetry.update();
//
//        // Ensure the robot is stationary, then reset the encoders and calibrate the gyro.
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
////        // Send telemetry message to alert driver that we are calibrating;
////        telemetry.addData(">", "Calibrating Gyro");    //
////        telemetry.update();
//
//        //robot.modernRoboticsI2cGyro.calibrate();
//
//
////        // make sure the gyro is calibrated before continuing
////        while (!isStopRequested()) {
////            sleep(50);
////            idle();
////        }
//
//        telemetry.addData(">", "Robot Ready.");    //
//        telemetry.update();
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        //robot.autoServoLeft.setPosition(0.62);
//        // Wait for the game to start (Display Gyro value), and reset gyro before we move.
//        while (!isStarted()) {
//            telemetry.addData(">", "Robot Heading1 = forward"); //%d", robot.modernRoboticsI2cGyro.getIntegratedZValue());
//            telemetry.update();
//        }
//        //robot.modernRoboticsI2cGyro.resetZAxisIntegrator();
//        //encoderDrive(1,22,-22,2);
//        FFTestCode();
//        //TestTest();
//        //autoPartA1();
//        //autoPartB();
//    }
//
//    public void FFTestCode(){
//        sleep(5000);
//        encoderDrive(0.2, 4.7,4.7,4);
//        int position = checkColor();
//        encoderStrafeRight(0.2, 0.55, 4);
//        encoderDrive(0.2,0.6,0.6,4);
//
//        double inches;
//        if(position == 3)
//        {
//            inches=3.65;
//        }
//        else if (position == 2)
//        {
//            inches = 2.65;
//        }
//        else
//        {
//            inches = 1.5;
//        }
//        encoderArm(-0.5,-inches,10, position);
////
//        encoderDrive(0.2, -0.7,-0.7,3);
//        encoderDrive(0.2,-6.4,6.4,3);
//        encoderDrive(0.2,15.13,15.13,4);
//        encoderDrive(0.2,-6.4,6.4,3);
//        encoderDrive(0.2,2.2,2.2,3);
//        duckC(-0.13,5);
//        encoderDrive(0.2,0,0,4);
//        robot.duck.setPower(0);
//        encoderDrive(0.2,-6.26,-6.26,4);
//
//
//
//
//
//    }
//    public void encoderArm(double speed, double inches, double timeoutS, int position) {
//        int newTarget;
//        resetMotors();
//        if (opModeIsActive()) {
//            newTarget = robot.arm.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
//            robot.arm.setTargetPosition(newTarget);
//            robot.intake.setTargetPosition(newTarget*4);
//            double inch;
//
//            //hsaY
//
//            encoderDrive(0.2,0,0,3);
//            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            runtime.reset();
//            robot.arm.setPower(Math.abs(speed*10));
//            //intake(-0.6,2);
//            sleep(100);
//            robot.intake.setPower(-speed*0.44);
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.arm.isBusy())) {
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :", newTarget);
//                telemetry.addData("Path2",  "Running at %7d :",
//                        robot.arm.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//
//            robot.arm.setPower(0.01);
//
//            // Turn off RUN_TO_POSITION
//            robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            sleep(250);   // optional pause after each move
//        }
//        resetMotors();
//    }
//
//
//
//    //    public void TestTest(){
////        intake(-0.7);
////        //gyroDrive(1,16,0);
////        resetMotors();
////        checkColor(0.4);
////        resetMotors();
////        encoderStrafeLeft(1,0.3,2);
////        resetMotors();
////        encoderDriveTwoSpeed(0.7,0.3,34,8,2);
////        resetMotors();
////        encoderDriveTwoSpeed(0.7,0.3,-34,-8,2);
////        resetMotors();
////        encoderDrive(1,-4,-4,2);
////        encoderDrive(1,-15,15,2);
////        resetMotors();
////        resetMotors();
////        //gyroDrive(1,-30,0);
////        resetMotors();
////        encoderDrive(1,-15,15,2);
////        resetMotors();
////        intake(0.5);
////
////        //gyroDrive(0.5,-12,0);
////        sleep(1000);
////        resetMotors();
////        encoderDrive(1,42,42,3);
////        //encoderStrafeLeft(1,1,2);
////        resetMotors();
////        encoderStrafeRight(1,3.5, 2);
////        //encoderStrafeLeft(1, 2.0,2);
////        //gyroTurn(1,185);
////
////    }
//    public void intake(double speed, double time){
//        runtime.reset();
//        while (runtime.seconds()>time) {
//            robot.intake.setPower(speed);
//        }
//    }
//    public void duckC(double speed, double time){
//        runtime.reset();
//        while (runtime.seconds()<time) {
//            robot.duck.setPower(speed);
//        }
//    }
////    public void autoPartA1(){
////        telemetry.addData("Motor speeds", "robot.frontLeft.getPower(), robot.frontRight.getPower(), robot.backLeft.getPower(), robot.backRight.getPower()", robot.backRight.getPower(), robot.backLeft.getPower(), robot.frontRight.getPower(), robot.frontLeft.getPower());
////        telemetry.update();
////        //Get close to blocks
////
////        resetMotors();
////        encoderStrafeLeft(1,2.55,3);
////        //Find color block
////        checkColor(0.4);
////        //get block
////        sleep(1000);
////        encoderStrafeRight(0.9,1.03,2);
////        resetMotors();
////
////        resetMotors();
////        encoderDrive(1, -17,-17,2);
////        resetMotors();
////        encoderDrive(1,17,17,2);
////
////        //move again to blocks
////        resetMotors();
////        resetMotors();
////        encoderStrafeLeft(1,1.13,2);
////        resetMotors();
////        sleep(1000);
////        encoderStrafeRight(0.8,1.23,2);
////        resetMotors();
////        //encoderDrive(0.85, -moveForBlock2, -moveForBlock2,2);
////        encoderDrive(1,-13,-13,2);
////
////        encoderDrive(2, -36,-36,2);
////        resetMotors();
////        encoderDrive(1, 24, -24,2);
////        //resetMotors();
////        //encoderStrafeRight(1, 0.5,2);
////        resetMotors();
////        encoderDrive(1,-22,-22,2);
////        sleep(800);
////        resetMotors();
////        encoderDrive(2, 44,44,2);
////        resetMotors();
////
////        encoderStrafeLeft(2, 4.5,2);
////        resetMotors();
////    }
//
////    public void checkColor(double speed){
////        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        //double distance = 0;
////        //int startPos = robot.frontRight.getCurrentPosition();
////
////        while (hsvValues[0] <= 90 ) {
////            robot.frontRight.setPower(speed);
////            robot.frontLeft.setPower(-speed);
////            robot.backRight.setPower(-speed);
////            robot.backLeft.setPower(speed);
////            Color.RGBToHSV((int) (robot.sensorColorDistanceLeft.red() * SCALE_FACTOR),
////                    (int) (robot.sensorColorDistanceLeft.green() * SCALE_FACTOR),
////                    (int) (robot.sensorColorDistanceLeft.blue() * SCALE_FACTOR),
////                    hsvValues);
////        }
////        //    robot.sensorColor.enableLed(false);
////        //distance = (robot.frontRight.getCurrentPosition() - startPos) * 1.0 / COUNTS_PER_INCH ;
////        robot.frontRight.setPower(0);
////        robot.frontLeft.setPower(0);
////        robot.backRight.setPower(0);
////        robot.backLeft.setPower(0);
////        //return distance;
////
////    }
//public int checkColor(){
//    //double distance = 0;
//    //int startPos = robot.frontRight.getCurrentPosition();
//
//        robot.sensorColorDistanceLeft.enableLed(false);
//        int left = robot.sensorColorDistanceLeft.red() +robot.sensorColorDistanceLeft.green()+robot.sensorColorDistanceLeft.blue();
//        int right = robot.sensorColorDistanceRight.red() + robot.sensorColorDistanceRight.green()+robot.sensorColorDistanceRight.blue();
//        Color.RGBToHSV((int) (robot.sensorColorDistanceLeft.red() * SCALE_FACTOR),
//                (int) (robot.sensorColorDistanceLeft.green() * SCALE_FACTOR),
//                (int) (robot.sensorColorDistanceLeft.blue() * SCALE_FACTOR),
//                hsvValues);
//    Color.RGBToHSV((int) (robot.sensorColorDistanceRight.red() * SCALE_FACTOR),
//            (int) (robot.sensorColorDistanceRight.green() * SCALE_FACTOR),
//            (int) (robot.sensorColorDistanceRight.blue() * SCALE_FACTOR),
//            hsvValues2);
//    telemetry.addData(">", left); //%d", robot.modernRoboticsI2cGyro.getIntegratedZValue());
//    telemetry.update();
//
//    if(left >= 500)
//    {
//        return 2;
//    }
//    else if(right >= 500)
//    {
//        return 3;
//    }
//    else
//    {
//        return 1;
//    }
//    //distance = (robot.frontRight.getCurrentPosition() - startPos) * 1.0 / COUNTS_PER_INCH ;
//    //return distance;
//
//}
//
//
//
//    public void resetMotors()
//    {
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }
///*
//    public void gyroDrive ( double speed,
//                            double distance,
//                            double angle) {
//
//        int     newLeftTarget;
//        int     newRightTarget;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//
//        speed *= 1.6;
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            moveCounts = (int) (distance * COUNTS_PER_INCH);
//            newLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
//            newRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
//
//            // Set Target and Turn On RUN_TO_POSITION
//            robot.frontLeft.setTargetPosition(newLeftTarget);
//            robot.frontRight.setTargetPosition(newRightTarget);
//            robot.backLeft.setTargetPosition(newLeftTarget);
//            robot.backRight.setTargetPosition(newRightTarget);
//
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // start motion.
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            robot.frontLeft.setPower(speed);
//            robot.frontRight.setPower(speed);
//            robot.backLeft.setPower(speed);
//            robot.backRight.setPower(speed);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
//
//                // adjust relative speed based on heading error.
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = speed - steer;
//                rightSpeed = speed + steer;
//
//                // Normalize speeds if either one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                robot.frontLeft.setPower(leftSpeed);
//                robot.frontRight.setPower(rightSpeed);
//                robot.backLeft.setPower(leftSpeed);
//                robot.backRight.setPower(rightSpeed);
//
//            }
//
//            // Stop all motion;
//            robot.frontLeft.setPower(0);
//            robot.frontRight.setPower(0);
//            robot.backLeft.setPower(0);
//            robot.backRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        resetMotors();
//    }
//    public void gyroStrafeRight ( double speed,
//                                  double distance,
//                                  double angle) {
//
//        int     newLeftTarget;
//        int     newRightTarget;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            moveCounts = (int) (distance * COUNTS_PER_INCH);
//            newLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
//            newRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
//
//            // Set Target and Turn On RUN_TO_POSITION
//            robot.frontLeft.setTargetPosition(newLeftTarget);
//            robot.frontRight.setTargetPosition(newRightTarget);
//            robot.backLeft.setTargetPosition(newLeftTarget);
//            robot.backRight.setTargetPosition(newRightTarget);
//
//
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // start motion.
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            robot.frontLeft.setPower(speed);
//            robot.frontRight.setPower(-speed);
//            robot.backLeft.setPower(-speed);
//            robot.backRight.setPower(speed);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
//
//                // adjust relative speed based on heading error.
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = speed - steer;
//                rightSpeed = speed + steer;
//
//                // Normalize speeds if either one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                robot.frontLeft.setPower(leftSpeed);
//                robot.frontRight.setPower(-rightSpeed);
//                robot.backLeft.setPower(-leftSpeed);
//                robot.backRight.setPower(rightSpeed);
//
//            }
//
//            // Stop all motion;
//            robot.frontLeft.setPower(0);
//            robot.frontRight.setPower(0);
//            robot.backLeft.setPower(0);
//            robot.backRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        resetMotors();
//    }
//
//    public void gyroStrafeLeft ( double speed,
//                                 double distance,
//                                 double angle) {
//
//        int     newLeftTarget;
//        int     newRightTarget;
//        int     moveCounts;
//        double  max;
//        double  error;
//        double  steer;
//        double  leftSpeed;
//        double  rightSpeed;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            moveCounts = (int) (distance * COUNTS_PER_INCH);
//            newLeftTarget = robot.frontLeft.getCurrentPosition() + moveCounts;
//            newRightTarget = robot.frontRight.getCurrentPosition() + moveCounts;
//
//            // Set Target and Turn On RUN_TO_POSITION
//            robot.frontLeft.setTargetPosition(newLeftTarget);
//            robot.frontRight.setTargetPosition(newRightTarget);
//            robot.backLeft.setTargetPosition(newLeftTarget);
//            robot.backRight.setTargetPosition(newRightTarget);
//
//
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // start motion.
//            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
//            robot.frontLeft.setPower(-speed);
//            robot.frontRight.setPower(speed);
//            robot.backLeft.setPower(speed);
//            robot.backRight.setPower(-speed);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
//
//                // adjust relative speed based on heading error.
//                error = getError(angle);
//                steer = getSteer(error, P_DRIVE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                leftSpeed = speed - steer;
//                rightSpeed = speed + steer;
//
//                // Normalize speeds if either one exceeds +/- 1.0;
//                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//                if (max > 1.0)
//                {
//                    leftSpeed /= max;
//                    rightSpeed /= max;
//                }
//
//                robot.frontLeft.setPower(-leftSpeed);
//                robot.frontRight.setPower(rightSpeed);
//                robot.backLeft.setPower(leftSpeed);
//                robot.backRight.setPower(-rightSpeed);
//
//            }
//
//            // Stop all motion;
//            robot.frontLeft.setPower(0);
//            robot.frontRight.setPower(0);
//            robot.backLeft.setPower(0);
//            robot.backRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//        resetMotors();
//    }
//*/
//    /**
//     *  Method to spin on central axis to point in a new direction.
//     *  Move will stop if either of these conditions occur:
//     *  1) Move gets to the heading (angle)
//     *  2) Driver stops the opmode running.
//     *
//     * @param speed Desired speed of turn.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     */
//    /*
//    public void gyroTurn (  double speed, double angle) {
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && onHeading(speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//        }
//        resetMotors();
//    }*/
//
//    /**
//     *  Method to obtain & hold a heading for a finite amount of time
//     *  Move will stop once the requested time has elapsed
//     *
//     * @param speed      Desired speed of turn.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     * @param holdTime   Length of time (in seconds) to hold the specified heading.
//     */
//    /*
//    public void gyroHold( double speed, double angle, double holdTime) {
//
//        ElapsedTime holdTimer = new ElapsedTime();
//
//        // keep looping while we have time remaining.
//        holdTimer.reset();
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//            // Update telemetry & Allow time for other processes to run.
//            onHeading(speed, angle, P_TURN_COEFF);
//            telemetry.update();
//        }
//
//        // Stop all motion;
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//        resetMotors();
//    }*/
//
//    /**
//     * Perform one cycle of closed loop heading control.
//     *
//     * @param speed     Desired speed of turn.
//     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
//     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                  If a relative angle is required, add/subtract from current heading.
//     * @param PCoeff    Proportional Gain coefficient
//     * @return
//     */
//    /*
//    boolean onHeading(double speed, double angle, double PCoeff) {
//        double   error ;
//        double   steer ;
//        boolean  onTarget = false ;
//        double leftSpeed;
//        double rightSpeed;
//
//        // determine turn power based on +/- error
//        error = getError(angle);
//
//        if (Math.abs(error) <= HEADING_THRESHOLD) {
//            steer = 0.0;
//            leftSpeed  = 0.0;
//            rightSpeed = 0.0;
//            onTarget = true;
//        }
//        else {
//            steer = getSteer(error, PCoeff);
//            rightSpeed  = speed * steer;
//            leftSpeed   = -rightSpeed;
//        }
//
//        // Send desired speeds to motors.
//        robot.frontLeft.setPower(leftSpeed);
//        robot.frontRight.setPower(rightSpeed);
//        robot.backLeft.setPower(leftSpeed);
//        robot.backRight.setPower(rightSpeed);
//
//
//        return onTarget;
//
//    }*/
//
//    /**
//     * getError determines the error between the target angle and the robot's current heading
//     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
//     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
//     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
//     */
//    /*
//    public double getError(double targetAngle) {
//
//        double robotError;
//
//        // calculate error in -179 to +180 range  (
//        robotError = targetAngle - robot.modernRoboticsI2cGyro.getIntegratedZValue();
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }*/
//
//    /**
//     * returns desired steering force.  +/- 1 range.  +ve = steer left
//     * @param error   Error angle in robot relative degrees
//     * @param PCoeff  Proportional Gain Coefficient
//     * @return
//     */
//    public double getSteer(double error, double PCoeff) {
//        return Range.clip(error * PCoeff, -1, 1);
//    }
//
//    public void encoderDriveTwoSpeed(double leftSpeed, double rightSpeed,
//                                     double leftInches, double rightInches,
//                                     double timeoutS) {
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//
//            robot.frontLeft.setTargetPosition(robot.frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
//            robot.backLeft.setTargetPosition(robot.backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
//            robot.frontRight.setTargetPosition(robot.frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));
//            robot.backRight.setTargetPosition(robot.backRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));
//
//
//            // Turn On RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.frontLeft.setPower(Math.abs(leftSpeed));
//            robot.backLeft.setPower(Math.abs(leftSpeed));
//            robot.frontRight.setPower(Math.abs(rightSpeed));
//            robot.backRight.setPower(Math.abs(rightSpeed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d");
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        robot.frontLeft.getCurrentPosition(),
//                        robot.backLeft.getCurrentPosition(),
//                        robot.frontRight.getCurrentPosition(),
//                        robot.backRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.frontLeft.setPower(0);
//            robot.backLeft.setPower(0);
//            robot.frontRight.setPower(0);
//            robot.backRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            sleep(250);   // optional pause after each move
//        }
//        resetMotors();
//    }
//
//
//
//
//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newLeftTarget = robot.backLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRightTarget = robot.frontRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            newRightTarget = robot.backRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            robot.frontLeft.setTargetPosition(newLeftTarget);
//            robot.backLeft.setTargetPosition(newLeftTarget);
//            robot.frontRight.setTargetPosition(newRightTarget);
//            robot.backRight.setTargetPosition(newRightTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.frontLeft.setPower(Math.abs(speed));
//            robot.backLeft.setPower(Math.abs(speed));
//            robot.frontRight.setPower(Math.abs(speed));
//            robot.backRight.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        robot.frontLeft.getCurrentPosition(),
//                        robot.backLeft.getCurrentPosition(),
//                        robot.frontRight.getCurrentPosition(),
//                        robot.backRight.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.frontLeft.setPower(0);
//            robot.backLeft.setPower(0);
//            robot.frontRight.setPower(0);
//            robot.backRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            sleep(250);   // optional pause after each move
//        }
//        resetMotors();
//    }
//
//    public void encoderStrafeRight(double speed, double inches, double timeoutS)
//    {
//        int newFrontRightTarget = robot.frontRight.getCurrentPosition() - ((int)(inches * COUNTS_PER_MOTOR_REV));
//        int newBackRightTarget = robot.backRight.getCurrentPosition() + ((int)(inches * COUNTS_PER_MOTOR_REV));
//
//        robot.frontLeft.setTargetPosition(newBackRightTarget);
//        robot.frontRight.setTargetPosition(newFrontRightTarget);
//        robot.backLeft.setTargetPosition(newFrontRightTarget);
//        robot.backRight.setTargetPosition(newBackRightTarget);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontLeft.setPower(-speed);
//        robot.frontRight.setPower(speed);
//        robot.backLeft.setPower(speed);
//        robot.backRight.setPower(-speed);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
//                && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d : %7d", newFrontRightTarget,newBackRightTarget);
//            telemetry.addData("Path2",  "Running at %7d : %7d : %7d : %7d", robot.frontLeft.getCurrentPosition(),robot.frontRight.getCurrentPosition(),robot.backLeft.getCurrentPosition(),robot.backRight.getCurrentPosition());
//            telemetry.update();
//        }
//
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        resetMotors();
//    }
//    public void encoderStrafeLeft(double speed, double revolutions, double timeoutS)
//    {
//        int newFrontLeftTarget = robot.frontLeft.getCurrentPosition() - (int)(revolutions * COUNTS_PER_MOTOR_REV);
//        int newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(revolutions * COUNTS_PER_MOTOR_REV);
//
//        robot.frontLeft.setTargetPosition(newFrontLeftTarget);
//        robot.frontRight.setTargetPosition(newBackLeftTarget);
//        robot.backLeft.setTargetPosition(newBackLeftTarget);
//        robot.backRight.setTargetPosition(newFrontLeftTarget);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontLeft.setPower(-speed);
//        robot.frontRight.setPower(speed);
//        robot.backLeft.setPower(speed);
//        robot.backRight.setPower(-speed);
//
//        runtime.reset();
//        while (opModeIsActive() && (runtime.seconds() < timeoutS) && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
//                && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d", newFrontLeftTarget);
//            telemetry.addData("Path2",  "Running at %7d", robot.frontLeft.getCurrentPosition());
//            telemetry.update();
//        }
//
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        resetMotors();
//    }
//
//    public void encoderSlide(double speed,
//                             double inches,
//                             double timeoutS) {
//        int newTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newTarget = robot.arm.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
//            robot.arm.setTargetPosition(newTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.arm.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.arm.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        robot.arm.getCurrentPosition(),
//                        telemetry.update());
//            }
//
//            // Stop all motion;
//            robot.arm.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            resetMotors();
//
//            sleep(250);   // optional pause after each move
//        }
//    }
//}
//
