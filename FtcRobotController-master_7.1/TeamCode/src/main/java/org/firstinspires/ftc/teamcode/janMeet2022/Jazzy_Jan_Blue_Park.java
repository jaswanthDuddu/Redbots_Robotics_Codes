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
//@Autonomous(name="HSAY_Blue_Park", group="Jaswanth")
//public class Jazzy_Jan_Blue_Park extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    Jazzy_Jan_Hardware   robot   = new Jazzy_Jan_Hardware();   // Use a Pushbot's hardware
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
//    final double SERVO_OPEN = 0.4;
//    final double SERVO_CLOSE = 0.85;
//
//    static final double SERVO_Open = 0.5;
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
//        telemetry.addData(">", "Calibrating Gyro");    //
//        telemetry.update();
//
//        robot.gyroMR.calibrate();
//
//
//        // make sure the gyro is calibrated before continuing
//        while (!isStopRequested() && robot.gyroMR.isCalibrating()) {
//            sleep(50);
//            idle();
//        }
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
//        robot.holder.setPosition(SERVO_CLOSE);
//
//        robot.gyroMR.resetZAxisIntegrator();
//        while (!isStarted()) {
//            telemetry.addData(">", "Robot Z-Axis = %d", robot.gyroMR.getIntegratedZValue());
//            telemetry.addData(">", "Robot x = %d", robot.gyroMR.rawX());
//            telemetry.addData(">", "Robot x = %d", robot.gyroMR.rawY());
//            telemetry.addData(">", "Robot x = %d", robot.gyroMR.rawZ());
//            telemetry.update();
//        }
//        robot.gyroMR.resetZAxisIntegrator();
//
//        park();
//    }
//    public void turnTest()
//    {
//        //gyroTurnTest(1,45);
//        encoderDrive(0.4,7,7,5,0);
//        //gyroDrive(0.5,10,0);
//        encoderDrive(0.5,-1,1,3,0);
//
//    }
//    public void JanMeet()
//    {
//        encoderDrive(0.3,-9,-9,4 , 6);
//        int position = checkColor();
//        telemetry.addData(">Position: ", position);
//        telemetry.update();
//        encoderStrafeRight(0.3,0.85,4);
//        //robot to hub distance is 2.75 inch
//        //level 1 is 0 inch
//        //Level 2 is -1.3 inch
//        //level 3 is -3.1 inch
//        if(position == 3)
//        {
//            //encoderDrive(0.3,-1.2,-1.2,3, 6);
//            encoderArm(1,-3.1,7);
//            servoGate(SERVO_OPEN);
//            //encoderArm(1,3.1,7);
//            sleep(1000);
//            servoGate(SERVO_CLOSE);
//            //encoderDrive(0.3,1.7,1.7,3 , 6);
//        }
//        else if(position == 2)
//        {
//            //encoderDrive(0.3,-0.95,-0.95,3 , 6);
//            encoderArm(1,-1.3,4.6);
//            servoGate(SERVO_OPEN);
//            //encoderArm(1,1.3,4.6);
//            sleep(1000);
//            servoGate(SERVO_CLOSE);
//            //encoderDrive(0.3,1.55,1.55,3 , 6);
//        }
//        else
//        {
//            //encoderDrive(0.3,-0.9,-0.9,3 , 6);
//            servoGate(SERVO_OPEN);
//            sleep(1000);
//            servoGate(SERVO_CLOSE);
//            //encoderDrive(0.3,1.5,1.5,3 , 6);
//        }
//        encoderStrafeLeft(0.5,2,7);
//        encoderDriveWithoutAngle(0.4,-7.605,7.605,4);
//        encoderDriveWithoutAngle(0.5,1.3,1.3,3);
//        encoderStrafeRight(0.5,0.75,3);
//        duckC(0.5,3);
//        encoderStrafeLeft(0.5,0.95,7);
//        encoderDriveWithoutAngle(0.7,3,3,3);
//        if(position == 3)
//        {
//            encoderArm(1,3.1,7);
//        }
//        else if(position == 2)
//        {
//            encoderArm(1,1.3,4.6);
//        }
//        else
//        {
//            sleep(100);
//        }
//
//    }
//    public void park()
//    {
//        encoderStrafeLeft(0.4,1.1,4);
//    }
////    public void tryTest()
////    {
////        double servoPlace = 0.695;
////        double servoHold = 0.5;
////        double servoOrig = 0.24;
////        double servoWorst = 0.8;
////        encoderArm(0.7,5.55,2);
////        encoderDrive(0.4,-8.6,-8.6,4);
////        int position = checkColor();
////        telemetry.addData("Position Level ", position);
////        telemetry.update();
////        encoderStrafeLeft(0.3,0.75,3);
////         //encoderDrive(0.2,-2,-2,2);
////        if(position == 3)
////        {
////            encoderDrive(0.4,-1.9,-1.9,3);
////            robot.holder.setPosition(servoPlace);
////            sleep(2000);
////            robot.holder.setPosition(servoOrig);
////            encoderDrive(0.4,4.4,4.4,4);
////        }
////        else if(position == 2)
////        {
////            robot.holder.setPosition(servoPlace);
////            sleep(2000);
////            robot.holder.setPosition(servoOrig);
////            encoderDrive(0.4,2.5,2.5,4);
////        }
////        else
////        {
////            encoderDrive(0.4,0.5,0.5,2);
////            robot.holder.setPosition(servoHold);
////            encoderArm(3,-4,1);
////            robot.holder.setPosition(servoPlace);
////            sleep(2000);
////            robot.holder.setPosition(servoHold);
////            encoderDrive(0.4,2,2,4);
////        }
////
////
////        encoderDrive(0.4,7.5,-7.5,4);
////        encoderDrive(0.4,22,22,4);
////        encoderStrafeLeft(0.2,0.3,3);
////        duckC(-0.21,4);
////        encoderStrafeRight(0.3,0.9,3);
////    }
////    public void Tst()
////    {
////        int position = checkColor();
////        sleep(3000);
////        telemetry.addData(">Position: ", position);
////        telemetry.update();
////        sleep(3000);
////
////    }
////
////    public void Test()
////    {
////        double servoPlace = 0.68;
////        double servoHold = 0.5;
////        double servoOrig = 0.25;
////        double servoWorst = 0.8;
////        encoderArm(3,5.6,2.1);
////        encoderDrive(0.4,-32.9,-32.9,3);
////        int position = checkColor();
////        telemetry.addData("Position Level ", position);
////        telemetry.update();
////        encoderStrafeLeft(0.3,3,3);
////        if(position == 3)
////        {
////            //encoderDrive(0.2,-2,-2,2);
////            robot.holder.setPosition(servoPlace);
////            sleep(2000);
////            //encoderDrive(0.2,-2,-2,2);
////        }
////        else if(position == 2)
////        {
////            robot.holder.setPosition(servoWorst);
////            sleep(2000);
////
//////            robot.holder.setPosition(servoHold);
//////            sleep(250);
//////            encoderArm(3,-0.8,2,position);
//////            robot.holder.setPosition(servoPlace);
//////            sleep(2000);
//////            encoderArm(3,0.3,2,position);
////        }
////        else
////        {
////            encoderDrive(0.2,2,2,2);
////            robot.holder.setPosition(servoHold);
////            encoderArm(3,-1.16,2.4);
////            robot.holder.setPosition(servoPlace);
////            sleep(2000);
////            robot.holder.setPosition(servoHold);
////            encoderArm(3,1.0,2);
////            encoderDrive(0.2,-1,-1,2);
////        }
////        robot.holder.setPosition(servoOrig);
////        encoderDrive(0.4,10,10,4);
////        encoderDrive(0.4,26,-26,4);
////        encoderDrive(0.5,80,80,4);
////        encoderStrafeLeft(0.2,1.5,3);
////        duckC(0.21,4);
////        encoderStrafeRight(0.3,3.5,3);
////    }
////    public void TestTest()
////    {
////        double servoPlace = 0.68;
////        double servoHold = 0.5;
////        double servoOrig = 0.25;
////        double servoWorst = 0.8;
////        double forwardDist = 0;
////
////        encoderDrive(0.4, -33.2,-33.2,3);
////        int position = checkColor();
////        telemetry.addData("Position Level ", position);
////        telemetry.update();
////        encoderStrafeLeft(0.2,3,4);
////        encoderArm(1,260,5);
////        if(position == 3)
////        {
////            robot.holder.setPosition(servoPlace);
////            sleep(3000);
////        }
////        else if(position == 2)
////        {
////            robot.holder.setPosition(servoHold);
////            encoderArm(1,-125,4);
////            robot.holder.setPosition(servoPlace);
////            sleep(2000);
////            encoderArm(1,100,4);
////            //encoderArm(0.8,50,4,position);
////        }
////        else
////        {
////            robot.holder.setPosition(servoHold);
////            encoderArm(1,-250,4);
////            robot.holder.setPosition(servoPlace);
////            sleep(2000);
////            robot.holder.setPosition(servoHold);
////            encoderArm(1,225,4);
////        }
////        robot.holder.setPosition(servoOrig);
////        encoderDrive(0.4,10,10,4);
////        encoderDrive(0.4,26,-26,4);
////        encoderDrive(0.5,80,80,4);
////        encoderStrafeLeft(0.2,1.5,3);
////        duckC(0.2,3);
////        encoderStrafeRight(0.3,3.5,3);
////    }
//
//    public void servoGate(double count)
//    {
//        robot.holder.setPosition(count);
//        sleep(200);
//    }
//    public void encoderArm(double speed, double inches, double timeoutS) {
//        int newTarget;
//        resetMotors();
//        if (opModeIsActive()) {
//            newTarget = robot.lifter.getCurrentPosition() + (int)((inches*10) * (COUNTS_PER_MOTOR_REV / (0.75 * 3.1415)));
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
//            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.lifter.isBusy())) {
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
//            //sleep(250);   // optional pause after each move
//        }
//        resetMotors();
//    }
//
//    public void intake(double speed, double time){
//        runtime.reset();
//        while (runtime.seconds()>time) {
//            robot.intake.setPower(speed);
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
//        robot.colorLeft.enableLed(true);
//        robot.colorRight.enableLed(true);
//        int left = robot.colorLeft.red() +robot.colorLeft.green()+robot.colorLeft.blue();
//        int right = robot.colorRight.red() + robot.colorRight.green()+robot.colorRight.blue();
//        Color.RGBToHSV((int) (robot.colorLeft.red() * SCALE_FACTOR),
//                (int) (robot.colorLeft.green() * SCALE_FACTOR),
//                (int) (robot.colorLeft.blue() * SCALE_FACTOR),
//                hsvValues);
//        Color.RGBToHSV((int) (robot.colorRight.red() * SCALE_FACTOR),
//                (int) (robot.colorRight.green() * SCALE_FACTOR),
//                (int) (robot.colorRight.blue() * SCALE_FACTOR),
//                hsvValues2);
//        telemetry.addData(">>>>Level 3: ", left);//%d", robot.modernRoboticsI2cGyro.getIntegratedZValue());
//        telemetry.addData(">>>>Level 2: ", right);
//        telemetry.update();
//        robot.colorLeft.enableLed(false);
//        robot.colorRight.enableLed(false);
//        robot.colorLeft.close();
//        robot.colorRight.close();
//        //sleep(5000);
//
//        if(left >= 400)
//        {
//            return 2;
//        }
//        else if(right >= 400)
//        {
//            return 1;
//        }
//        else
//        {
//            return 3;
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
//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS, double angle) {
//        int newLeftTarget;
//        int newRightTarget;
//        robot.gyroMR.resetZAxisIntegrator();
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
//            if(angle >= 0) {
//                while (opModeIsActive() &&
//                        (runtime.seconds() < timeoutS) &&
//                        (robot.frontLeft.isBusy() && robot.frontRight.isBusy()) && (angle >= robot.gyroMR.getIntegratedZValue())) {
//
//                    // Display it for the driver.
//                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                    telemetry.addData("Path2", "Running at %7d :%7d",
//                            robot.frontLeft.getCurrentPosition(),
//                            robot.backLeft.getCurrentPosition(),
//                            robot.frontRight.getCurrentPosition(),
//                            robot.backRight.getCurrentPosition());
//                    telemetry.update();
//                }
//            }
//            else
//            {
//                while (opModeIsActive() &&
//                        (runtime.seconds() < timeoutS) &&
//                        (robot.frontLeft.isBusy() && robot.frontRight.isBusy()) && (angle <= robot.gyroMR.getIntegratedZValue())) {
//
//                    // Display it for the driver.
//                    telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                    telemetry.addData("Path2", "Running at %7d :%7d",
//                            robot.frontLeft.getCurrentPosition(),
//                            robot.backLeft.getCurrentPosition(),
//                            robot.frontRight.getCurrentPosition(),
//                            robot.backRight.getCurrentPosition());
//                    telemetry.update();
//                }
//            }
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
//    public void encoderDriveWithoutAngle(double speed,
//                                         double leftInches, double rightInches,
//                                         double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//        robot.gyroMR.resetZAxisIntegrator();
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
//
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
//                telemetry.addData("Path2", "Running at %7d :%7d",
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
////    public void FFTestCode(){
////        encoderDrive(0.2, 4.75,4.75,4);
////        int position = checkColor();
////        encoderStrafeLeft(0.2, 0.55, 4);
////        encoderDrive(0.2,0,0,4);
////
////        double inches;
////        if(position == 3)
////        {
////            inches=3.65;
////        }
////        else if (position == 2)
////        {
////            inches = 2.65;
////        }
////        else
////        {
////            inches = 1.5;
////        }
////        encoderArm(-0.5,-inches,10);
////
////        encoderDrive(0.2, -0.85,-0.85,3);
////        encoderDrive(0.2,6.3,-6.3,3);
////        encoderDrive(0.2,15.5,15.5,4);
////        encoderStrafeRight(0.2,0.13,4);
////        duckC(0.13,5);
////        encoderDrive(0.2,0,0,4);
////        robot.duck.setPower(0);
////        encoderStrafeLeft(0.2,0.53,4);
////    }
//
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
//    }
//
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
//        robot.gyroMR.resetZAxisIntegrator();
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && onHeading(speed, angle, P_TURN_COEFF)) {
//            // Update telemetry & Allow time for other processes to run.
//            telemetry.update();
//        }
//    }
//    public void gyroTurnTest (  double speed, double angle) {
//        // keep looping while we are still active, and not on heading.
//        double angleZ = robot.gyroMR.getIntegratedZValue();
//
//        while (opModeIsActive() && angle >= angleZ) {
//            // Update telemetry & Allow time for other processes to run.
//            robot.frontLeft.setPower(-speed);
//            robot.backLeft.setPower(-speed);
//            robot.frontRight.setPower(speed);
//            robot.backRight.setPower(speed);
//            angleZ = robot.gyroMR.getIntegratedZValue();
//            telemetry.addData("Angle: ", angleZ);
//            telemetry.update();
//        }
//        robot.frontLeft.setPower(0);
//        robot.backLeft.setPower(0);
//        robot.frontRight.setPower(0);
//        robot.backRight.setPower(0);
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
//            rightSpeed  = speed * steer*10;
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
//        robotError = targetAngle - robot.gyroMR.getIntegratedZValue();
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
//}
