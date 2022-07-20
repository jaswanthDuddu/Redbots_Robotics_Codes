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
//public class Jazzy_Dec_Eight_Red extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    JazzyDecEightHardware   robot   = new JazzyDecEightHardware();   // Use a Pushbot's hardware
//    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device\
//    private ElapsedTime     runtime = new ElapsedTime();
//
//    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
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
//        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        telemetry.addData(">", "Robot Ready.");    //
//        telemetry.update();
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        robot.turner.setPosition(0.24);
//
//        while (!isStarted()) {
//            telemetry.addData(">", "Robot Heading1 = backward");
//            telemetry.update();
//        }
//
//        //TestTest();
//        //Test();
//        //Tst();
//        //tryTest();
//        park();
//    }
//    public void park()
//    {
//        encoderStrafeRight(0.4,1,3);
//    }
//    public void tryTest()
//    {
//        double servoPlace = 0.695;
//        double servoHold = 0.5;
//        double servoOrig = 0.24;
//        double servoWorst = 0.8;
//        encoderArm(0.7,5.55,2,2);
//        encoderDrive(0.4,-8.6,-8.6,4);
//        int position = checkColor();
//        telemetry.addData("Position Level ", position);
//        telemetry.update();
//        encoderStrafeLeft(0.3,0.75,3);
//         //encoderDrive(0.2,-2,-2,2);
//        if(position == 3)
//        {
//            encoderDrive(0.4,-1.9,-1.9,3);
//            robot.turner.setPosition(servoPlace);
//            sleep(2000);
//            robot.turner.setPosition(servoOrig);
//            encoderDrive(0.4,4.4,4.4,4);
//        }
//        else if(position == 2)
//        {
//            robot.turner.setPosition(servoPlace);
//            sleep(2000);
//            robot.turner.setPosition(servoOrig);
//            encoderDrive(0.4,2.5,2.5,4);
//        }
//        else
//        {
//            encoderDrive(0.4,0.5,0.5,2);
//            robot.turner.setPosition(servoHold);
//            encoderArm(3,-4,1,1);
//            robot.turner.setPosition(servoPlace);
//            sleep(2000);
//            robot.turner.setPosition(servoHold);
//            encoderDrive(0.4,2,2,4);
//        }
//
//
//        encoderDrive(0.4,7.5,-7.5,4);
//        encoderDrive(0.4,22,22,4);
//        encoderStrafeLeft(0.2,0.3,3);
//        duckC(-0.21,4);
//        encoderStrafeRight(0.3,0.9,3);
//    }
//    public void Tst()
//    {
//        int position = checkColor();
//        sleep(3000);
//        telemetry.addData(">", position);
//        telemetry.update();
//        sleep(3000);
//
//    }
//
//    public void Test()
//    {
//        double servoPlace = 0.68;
//        double servoHold = 0.5;
//        double servoOrig = 0.25;
//        double servoWorst = 0.8;
//        encoderArm(3,5.6,2.1,2);
//        encoderDrive(0.4,-32.9,-32.9,3);
//        int position = checkColor();
//        telemetry.addData("Position Level ", position);
//        telemetry.update();
//        encoderStrafeLeft(0.3,3,3);
//        if(position == 3)
//        {
//            //encoderDrive(0.2,-2,-2,2);
//            robot.turner.setPosition(servoPlace);
//            sleep(2000);
//            //encoderDrive(0.2,-2,-2,2);
//        }
//        else if(position == 2)
//        {
//            robot.turner.setPosition(servoWorst);
//            sleep(2000);
//
////            robot.turner.setPosition(servoHold);
////            sleep(250);
////            encoderArm(3,-0.8,2,position);
////            robot.turner.setPosition(servoPlace);
////            sleep(2000);
////            encoderArm(3,0.3,2,position);
//        }
//        else
//        {
//            encoderDrive(0.2,2,2,2);
//            robot.turner.setPosition(servoHold);
//            encoderArm(3,-1.16,2.4,position);
//            robot.turner.setPosition(servoPlace);
//            sleep(2000);
//            robot.turner.setPosition(servoHold);
//            encoderArm(3,1.0,2,position);
//            encoderDrive(0.2,-1,-1,2);
//        }
//        robot.turner.setPosition(servoOrig);
//        encoderDrive(0.4,10,10,4);
//        encoderDrive(0.4,26,-26,4);
//        encoderDrive(0.5,80,80,4);
//        encoderStrafeLeft(0.2,1.5,3);
//        duckC(0.21,4);
//        encoderStrafeRight(0.3,3.5,3);
//    }
//    public void TestTest()
//    {
//        double servoPlace = 0.68;
//        double servoHold = 0.5;
//        double servoOrig = 0.25;
//        double servoWorst = 0.8;
//        double forwardDist = 0;
//
//        encoderDrive(0.4, -33.2,-33.2,3);
//        int position = checkColor();
//        telemetry.addData("Position Level ", position);
//        telemetry.update();
//        encoderStrafeLeft(0.2,3,4);
//        encoderArm(1,260,5,position);
//        if(position == 3)
//        {
//            robot.turner.setPosition(servoPlace);
//            sleep(3000);
//        }
//        else if(position == 2)
//        {
//            robot.turner.setPosition(servoHold);
//            encoderArm(1,-125,4,position);
//            robot.turner.setPosition(servoPlace);
//            sleep(2000);
//            encoderArm(1,100,4,position);
//            //encoderArm(0.8,50,4,position);
//        }
//        else
//        {
//            robot.turner.setPosition(servoHold);
//            encoderArm(1,-250,4,position);
//            robot.turner.setPosition(servoPlace);
//            sleep(2000);
//            robot.turner.setPosition(servoHold);
//            encoderArm(1,225,4,position);
//        }
//        robot.turner.setPosition(servoOrig);
//        encoderDrive(0.4,10,10,4);
//        encoderDrive(0.4,26,-26,4);
//        encoderDrive(0.5,80,80,4);
//        encoderStrafeLeft(0.2,1.5,3);
//        duckC(0.2,3);
//        encoderStrafeRight(0.3,3.5,3);
//    }
//
//    public void encoderArm(double speed, double inches, double timeoutS, int position) {
//        int newTarget;
//        resetMotors();
//        if (opModeIsActive()) {
//            newTarget = robot.lifter.getCurrentPosition() + (int)((inches*10) * COUNTS_PER_INCH);
//            robot.lifter.setTargetPosition(newTarget);
//            double inch;
//
//            //hsaY
//
//            //encoderDrive(0.2,0,0,3);
//            robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            runtime.reset();
//            robot.lifter.setPower(Math.abs(speed*10));
//            //intake(-0.6,2);
//            sleep(700);
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.lifter.isBusy())) {
//                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :", newTarget);
//                telemetry.addData("Path2",  "Running at %7d :",
//                        robot.lifter.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//
//            robot.lifter.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            sleep(250);   // optional pause after each move
//        }
//        resetMotors();
//    }
//
//    public void intake(double speed, double time){
//        runtime.reset();
//        while (runtime.seconds()>time) {
//            robot.intakeLeft.setPower(speed);
//            robot.intakeRight.setPower(speed);
//        }
//    }
//
//    public void duckC(double speed, double time){
//        runtime.reset();
//        while (runtime.seconds()<time) {
//            robot.duck.setPower(speed);
//        }
//    }
//
//    public int checkColor(){
//
//        //robot.sensorColorDistanceLeft.enableLed(false);
//        int left = robot.sensorColorDistanceLeft.red() +robot.sensorColorDistanceLeft.green()+robot.sensorColorDistanceLeft.blue();
//        int right = robot.sensorColorDistanceRight.red() + robot.sensorColorDistanceRight.green()+robot.sensorColorDistanceRight.blue();
//        Color.RGBToHSV((int) (robot.sensorColorDistanceLeft.red() * SCALE_FACTOR),
//                (int) (robot.sensorColorDistanceLeft.green() * SCALE_FACTOR),
//                (int) (robot.sensorColorDistanceLeft.blue() * SCALE_FACTOR),
//                hsvValues);
//        Color.RGBToHSV((int) (robot.sensorColorDistanceRight.red() * SCALE_FACTOR),
//                (int) (robot.sensorColorDistanceRight.green() * SCALE_FACTOR),
//                (int) (robot.sensorColorDistanceRight.blue() * SCALE_FACTOR),
//                hsvValues2);
//        telemetry.addData(">>>>Level 3: ", left);//%d", robot.modernRoboticsI2cGyro.getIntegratedZValue());
//        telemetry.addData(">>>>Level 2: ", right);
//        telemetry.update();
//
//        if(left >= 90)
//        {
//            return 3;
//        }
//        else if(right >= 90)
//        {
//            return 2;
//        }
//        else
//        {
//            return 1;
//        }
//
//    }
//
//
//
//    public void resetMotors()
//    {
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
//    public void encoderStrafeLeftWithLifterUp(double speed, double revolutions,
//                                              double timeoutS,
//                                              double armSpeed, double liftInch)
//    {
//        int newFrontLeftTarget = robot.frontLeft.getCurrentPosition() - (int)(revolutions * COUNTS_PER_MOTOR_REV);
//        int newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(revolutions * COUNTS_PER_MOTOR_REV);
//        int newTarget = robot.lifter.getCurrentPosition() + (int)(liftInch * COUNTS_PER_INCH);
//
//        robot.lifter.setTargetPosition(newTarget);
//        robot.frontLeft.setTargetPosition(newFrontLeftTarget);
//        robot.frontRight.setTargetPosition(newBackLeftTarget);
//        robot.backLeft.setTargetPosition(newBackLeftTarget);
//        robot.backRight.setTargetPosition(newFrontLeftTarget);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.frontLeft.setPower(-speed);
//        robot.frontRight.setPower(speed);
//        robot.backLeft.setPower(speed);
//        robot.backRight.setPower(-speed);
//        robot.lifter.setPower(Math.abs(armSpeed*10));
//        int i = 0;
//        runtime.reset();
//
////        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.frontLeft.isBusy() && robot.frontRight.isBusy()
////                && robot.backLeft.isBusy() && robot.backRight.isBusy() || (robot.lifter.isBusy()))) {
////            // Display it for the driver.
////            telemetry.addData("Path1",  "Running to %7d", newFrontLeftTarget);
////            telemetry.addData("Path2",  "Running at %7d", robot.frontLeft.getCurrentPosition());
////            telemetry.addData("Path3",  "Running at %7d :", robot.lifter.getCurrentPosition());
////            telemetry.addData("Times ",  i);
////            i++;
////            telemetry.update();
////        }
//        while (opModeIsActive() && (runtime.seconds() < timeoutS) ) {
//            // Display it for the driver.
//            telemetry.addData("Path1",  "Running to %7d", newFrontLeftTarget);
//            telemetry.addData("Path2",  "Running at %7d", robot.frontLeft.getCurrentPosition());
//            telemetry.addData("Path3",  "Running at %7d :", robot.lifter.getCurrentPosition());
//            telemetry.addData("Times ",  i);
//            i++;
//            telemetry.update();
//        }
//        robot.frontLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.backRight.setPower(0);
//        robot.lifter.setPower(0);
//
//        // Turn off RUN_TO_POSITION
//        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        resetMotors();
//        sleep(250);
//    }
//    public void FFTestCode(){
//        encoderDrive(0.2, 4.75,4.75,4);
//        int position = checkColor();
//        encoderStrafeLeft(0.2, 0.55, 4);
//        encoderDrive(0.2,0,0,4);
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
//
//        encoderDrive(0.2, -0.85,-0.85,3);
//        encoderDrive(0.2,6.3,-6.3,3);
//        encoderDrive(0.2,15.5,15.5,4);
//        encoderStrafeRight(0.2,0.13,4);
//        duckC(0.13,5);
//        encoderDrive(0.2,0,0,4);
//        robot.duck.setPower(0);
//        encoderStrafeLeft(0.2,0.53,4);
//    }
//}
