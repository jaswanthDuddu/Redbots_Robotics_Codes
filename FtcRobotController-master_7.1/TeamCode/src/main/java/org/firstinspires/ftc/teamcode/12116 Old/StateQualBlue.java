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
//import android.app.Activity;
//import android.graphics.Color;
//import android.view.View;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//import java.util.Locale;
//
///**
// * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
// * It uses the common Pushbot hardware class to define the drive on the robot.
// * The code is structured as a LinearOpMode
// *
// * The code REQUIRES that you DO have encoders on the wheels,
// *   otherwise you would use: PushbotAutoDriveByTime;
// *
// *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
// *   otherwise you would use: PushbotAutoDriveByEncoder;
// *
// *  This code requires that the drive Motors have been configured such that a positive
// *  power command moves them forward, and causes the encoders to count UP.
// *
// *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
// *
// *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
// *  This is performed when the INIT button is pressed on the Driver Station.
// *  This code assumes that the robot is stationary when the INIT button is pressed.
// *  If this is not the case, then the INIT should be performed again.
// *
// *  Note: in this example, all angles are referenced to the initial coordinate frame set during the
// *  the Gyro Calibration process, or whenever the program issues a resetZAxisIntegrator() call on the Gyro.
// *
// *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
// *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
// *  This is consistent with the FTC field coordinate conventions set out in the document:
// *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// *
// * blue tape Hue value =
// */
//
//@Autonomous(name="Skystone BLUE", group="Yash")
////@Disabled
//public class StateQualBlue extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    HardwareAhen2         robot   = new HardwareAhen2();   // Use a Pushbot's hardware
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device\
//    private ElapsedTime     runtime = new ElapsedTime();
//
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
//    // sometimes it helps to multiply the raw RGB values with a scale factor
//    // to amplify/attentuate the measured values.
//    final double SCALE_FACTOR = 255;
//
//    // get a reference to the RelativeLayout so we can change the background
//    // color of the Robot Controller app to match the hue detected by the RGB sensor.
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        /*
//         * Initialize the standard drive system variables.
//         * The init() method of the hardware class does most of the work here
//         */
//        robot.init(hardwareMap);
//
//
//        telemetry.update();
//
//        // Ensure the robot is stationary, then reset the encoders and calibrate the gyro.
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.tray.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Send telemetry message to alert driver that we are calibrating;
//        telemetry.addData(">", "Calibrating Gyro");    //
//        telemetry.update();
//
//        robot.modernRoboticsI2cGyro.calibrate();
//
//
//        // make sure the gyro is calibrated before continuing
//        while (!isStopRequested() && robot.modernRoboticsI2cGyro.isCalibrating()) {
//            sleep(50);
//            idle();
//        }
//        robot.autoServo.setPosition(0.8);
//
//        telemetry.addData(">", "Robot Ready.");    //
//        telemetry.update();
//        robot.autoServo.setPosition(0.8);
//
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.tray.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        telemetry.addData(">", "Ahen says Run Using Encoder.");
//        robot.autoServo.setPosition(0.8);
//
//        // Wait for the game to start (Display Gyro value), and reset gyro before we move.
//        while (!isStarted()) {
//            telemetry.addData(">", "Robot Heading = %d", robot.modernRoboticsI2cGyro.getIntegratedZValue());
//            telemetry.update();
//        }
//
//        robot.modernRoboticsI2cGyro.resetZAxisIntegrator();
//
//
////            encoderStrafeLeft();
////
////            robot.frontRight.setPower(0.08);
////            robot.frontLeft.setPower(0.08);
////            robot.backRight.setPower(0.08);
////            robot.backLeft.setPower(0.08);
////            encoderDrive(0.05,20.0,20.0,5);
//
//        encoderDrive(DRIVE_SPEED, 30, 30,10);    // Drive FWD 30 inches
//
//        robot.autoServo.setPosition(0.8);
//
////        robot.frontLeft.setPower(-DRIVE_SPEED);
////        robot.frontRight.setPower(DRIVE_SPEED);
////        robot.backLeft.setPower(DRIVE_SPEED);
////        robot.backRight.setPower(-DRIVE_SPEED);
//
//
//
//        robot.frontRight.setPower(0);
//        robot.frontLeft.setPower(0);
//        robot.backRight.setPower(0);
//        robot.backLeft.setPower(0);
//
//        robot.autoServo.setPosition(0.32);
//
//        sleep(1000);
//
//        encoderDrive(DRIVE_SPEED,-15,-15,3);
//
//        encoderStrafeRight(DRIVE_SPEED,3,5);
//
////        encoderDrive(DRIVE_SPEED, 20,-20,3);
//
////        encoderDrive(DRIVE_SPEED,40,40,3);
//
//        robot.autoServo.setPosition(0.8);
//
//        encoderDrive(DRIVE_SPEED,-64,-64,10);
//
//        encoderDrive(DRIVE_SPEED, -20,20,10);
//
//        encoderDrive(DRIVE_SPEED,8,8,10);
//
//        robot.autoServo.setPosition(0.32);
//
//        encoderDrive(DRIVE_SPEED,-8,-8,10);
//
//        encoderDrive(DRIVE_SPEED, 20,-20,10);
//
//        encoderDrive(DRIVE_SPEED,64,64,10);
//
//        robot.autoServo.setPosition(0.8);
//
//        encoderDrive(DRIVE_SPEED,-8,-8,10);
//
//        sleep(500);
//
//    }
//    // Step through each leg of the path,
//    // Note: Reverse movement is obtained by setting a negative distance (not speed)
//    // Put a hold after each turn
////
//
//    public void encoderStrafeRight(double speed, double inches, double timeoutS) throws InterruptedException{
//
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
//    }
//    public void encoderStrafeLeft(double speed, double revolutions, double timeoutS) throws InterruptedException{
//
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
//    }
//    /**
//     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
//     *  Move will stop if either of these conditions occur:
//     *  1) Move gets to the desired position
//     *  2) Driver stops the opmode running.
//     *
//     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
//     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
//     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     */
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
//                // Display drive status for the driver.
//                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
//                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Actual", "%7d:%7d", robot.frontLeft.getCurrentPosition(),
//                        robot.frontRight.getCurrentPosition());
//                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//                telemetry.update();
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
//                // Display drive status for the driver.
//                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
//                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Actual", "%7d:%7d", robot.frontLeft.getCurrentPosition(),
//                        robot.frontRight.getCurrentPosition());
//                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//                telemetry.update();
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
//                // Display drive status for the driver.
//                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
//                telemetry.addData("Target", "%7d:%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Actual", "%7d:%7d", robot.frontLeft.getCurrentPosition(),
//                        robot.frontRight.getCurrentPosition());
//                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//                telemetry.update();
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
//    }
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
//    public void gyroTurn (  double speed, double angle) {
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//        }
//    }
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
//    }
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
//        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
//
//        return onTarget;
//    }
//
//    /**
//     * getError determines the error between the target angle and the robot's current heading
//     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
//     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
//     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
//     */
//    public double getError(double targetAngle) {
//
//        double robotError;
//
//        // calculate error in -179 to +180 range  (
//        robotError = targetAngle - robot.modernRoboticsI2cGyro.getIntegratedZValue();
//        while (robotError > 180)  robotError -= 360;
//        while (robotError <= -180) robotError += 360;
//        return robotError;
//    }
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
//    }
//    public void trayMover(double speed,
//                          double Inches,
//                          double timeoutS) {
//        int newTarget;
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newTarget = robot.tray.getCurrentPosition() + (int)(Inches * COUNTS_PER_INCH);
//
//            robot.tray.setTargetPosition(newTarget);
//
//
//            // Turn On RUN_TO_POSITION
//            robot.tray.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            robot.tray.setPower(Math.abs(speed));
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
//                telemetry.addData("Path1",  "Running to %7d :%7d", newTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        robot.tray.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            robot.tray.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.tray.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            sleep(250);   // optional pause after each move
//        }
//    }
//}
