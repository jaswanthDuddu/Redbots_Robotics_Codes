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
////import com.disnodeteam.dogecv.CameraViewDisplay;
////import com.disnodeteam.dogecv.DogeCV;
////import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
////import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
//import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ReadWriteFile;
//import org.firstinspires.ftc.robotcore.external.Func;
//
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
//
//
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
//import java.util.Locale;
//import org.firstinspires.ftc.robotcore.external.Func;
//import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
//
//import java.io.File;
//import java.util.Locale;
//import android.app.Activity;
//import android.view.View;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
////import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
//
//
///**
// * This is NOT an opmode.
// *
// * This class can be used to define all the specific hardware for a single robot.
// * In this case that robot is a Pushbot.
// * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
// *
// * This hardware class assumes the following device names have been configured on the robot:
// * Note:  All names are lower case and some have single spaces between words.
// *
// * Motor channel:  Left  drive motor:        "left_drive"
// * Motor channel:  Right drive motor:        "right_drive"
// * Motor channel:  Manipulator drive motor:  "left_arm"
// * Servo channel:  Servo to open left claw:  "left_hand"
// * Servo channel:  Servo to open right claw: "right_hand"
// */
//public class Jazzy_LT_Hardware
//{
//
////    static final double MARKER_RETRACTED = 0.45;
////    static final double MARKER_EXTENDED = 0.9;
////
////    public SamplingOrderDetector detector;
//
//    /* Public OpMode members. */
//    public DcMotor  frontLeft        = null;    //C0
//    public DcMotor  frontRight       = null;    //C1
//    public DcMotor  backLeft         = null;    //C2
//    public DcMotor  backRight        = null;    //C3
//    public DcMotor  intake           = null;    //E2
//    public DcMotor  lifter           = null;    //E0
//    public DcMotor  duck             = null;    //E3
//    public DcMotor  capHand          = null;    //E1
//    public Servo    holder           = null;    //C0
//    public Servo    capArm           = null;    //C5
//    ColorSensor colorLeft;        //C1
//    ColorSensor colorRight;       //C2
//
//
//
//    BNO055IMU imu;
//    Orientation angles;
//    Acceleration gravity;
//
//
//
//
//
////    ModernRoboticsI2cRangeSensor range;
////    IntegratingGyroscope gyro;
////    ModernRoboticsI2cGyro modernRoboticsI2cGyro;
////
////    IntegratingGyroscope gyro2;
////    ModernRoboticsI2cGyro modernRoboticsI2cGyro2;
////    public DigitalChannel digitalTouch;
////
////    DistanceSensor sensorDistanceBottom;
//
////      rjberry@rjcontrol.com
//
//
//
//    /* local OpMode members. */
//    HardwareMap hwMap           =  null;
//    private ElapsedTime period  = new ElapsedTime();
//
//    /* Constructor */
//    public Jazzy_LT_Hardware(){ }
//
//    /* Initialize standard Hardware interfaces */
//    public void init(HardwareMap ahwMap) {
//        // Save reference to Hardware map
//        hwMap = ahwMap;
//
//        // Define and Initialize Motors
//        frontLeft      = hwMap.get(DcMotor.class, "frontLeft");
//        frontRight     = hwMap.get(DcMotor.class, "frontRight");
//        backLeft       = hwMap.get(DcMotor.class, "backLeft");
//        backRight      = hwMap.get(DcMotor.class, "backRight");
//        lifter         = hwMap.get(DcMotor.class, "lifter");
//        intake         = hwMap.get(DcMotor.class, "intake");
//        duck           = hwMap.get(DcMotor.class, "duck");
//
//        capHand    = hwMap.get(DcMotor.class, "hand");
////
//        capArm         = hwMap.get(Servo.class, "arm");
//
//        holder         = hwMap.get(Servo.class, "holder");
////
//        //modernRoboticsI2cGyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
//        //gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
//
//        colorRight    = hwMap.get(ColorSensor.class, "colorRight");
//        colorLeft     = hwMap.get(ColorSensor.class, "colorLeft");
//
////        gyroMR = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
////        gyro = (IntegratingGyroscope)gyroMR;
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        imu = hwMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        gravity  = imu.getGravity();
//
//
//
//        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
//        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//        intake.setDirection(DcMotorSimple.Direction.FORWARD);
//        lifter.setDirection(DcMotorSimple.Direction.REVERSE);
//        duck.setDirection(DcMotorSimple.Direction.FORWARD);
//        capHand.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Set to REVERSE if using AndyMark motors
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Set to FORWARD if using AndyMark motors
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Set to REVERSE if using AndyMark motors
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Set to FORWARD if using AndyMark motors
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        capHand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        holder.setDirection(Servo.Direction.FORWARD);
////        capHand.setDirection(Servo.Direction.FORWARD);
//        capArm.setDirection(Servo.Direction.REVERSE);
//
//        // Set all motors to zero power
//        frontLeft.setPower(0);
//        frontRight.setPower(0);
//        backLeft.setPower(0);
//        backRight.setPower(0);
//        intake.setPower(0);
//        lifter.setPower(0);
//        duck.setPower(0);
//        capHand.setPower(0);
//
//
//        holder.setPosition(0.85);
//        //capArm.setPosition(0);
////        capHand.setPosition(0);
//
//        // Set all motors to run without encoders.
//        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        capHand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
////        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        float hsvValues[] = {0F, 0F, 0F};
//
//        // values is a reference to the hsvValues array.
//        final float values[] = hsvValues;
//
//        // sometimes it helps to multiply the raw RGB values with a scale factor
//        // to amplify/attentuate the measured values.
//        final double SCALE_FACTOR = 255;
//
//        // get a reference to the RelativeLayout so we can change the background
//        // color of the Robot Controller app to match the hue detected by the RGB sensor.
//        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
//        final View relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);
//    }
//
//    public void setAllPower(double p)
//    {
//        setMotorPower(p,p,p,p);
//    }
//
//    public void setMotorPower(double frontLeftSpeed, double frontRightSpeed, double backRightSpeed, double backLeftSpeed)
//    {
//        frontLeft.setPower(frontLeftSpeed);
//        frontRight.setPower(frontRightSpeed);
//        backRight.setPower(backRightSpeed);
//        backLeft.setPower(backLeftSpeed);
//
//    }
//}
//
///* stuff to do in order of importance
//
//- take pictures of gyro angular velocity telemetry and gyro straight correction for notebook
//- test ConceptRampMotorSpeed (just make sure it works)
//- fix gyro straight/turn coefficients (make it smoother) (depending on time)
//- test touch sensor (depending on time)
//
//- phone angle for SamplingOrderDetector
//- encoder unlatching
//- accurate mineral alignment maybe using GoldAlignDetector?
//- team marker. either a physical mechanism or robot has to actually go around?
//- touching crater (using Vuforia??)
//
//- all the above for the Crater side
//
//- limit switch (which is programmed just like a REV touch sensor (SensorDigitalTouch))
//- gyro strafe
//- switch front and back maybe
//- lerp / accelerate motors when starting motion
//
//*/
//
