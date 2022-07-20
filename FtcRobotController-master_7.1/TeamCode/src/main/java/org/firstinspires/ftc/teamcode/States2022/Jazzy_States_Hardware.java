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

import android.app.Activity;
import android.view.View;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;
import androidx.annotation.NonNull;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;



public class Jazzy_States_Hardware
{


    public DcMotor  frontLeft        = null;    //C0
    public DcMotor  frontRight       = null;    //C1
    public DcMotor  backLeft         = null;    //C2
    public DcMotor  backRight        = null;    //C3
    public DcMotor  intake           = null;    //E2
    public DcMotor  lifter           = null;    //E0
    public DcMotor  duck             = null;    //E3
    public DcMotor  capHand          = null;    //E1
    public Servo    holder           = null;    //C0
    public Servo    capArm           = null;    //C5
    ColorSensor colorLeft;        //C1
    ColorSensor colorRight;       //C2



    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;


//      rjberry@rjcontrol.com



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Jazzy_States_Hardware(){ }


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft      = hwMap.get(DcMotor.class, "frontLeft");
        frontRight     = hwMap.get(DcMotor.class, "frontRight");
        backLeft       = hwMap.get(DcMotor.class, "backLeft");
        backRight      = hwMap.get(DcMotor.class, "backRight");
        lifter         = hwMap.get(DcMotor.class, "lifter");
        intake         = hwMap.get(DcMotor.class, "intake");
        duck           = hwMap.get(DcMotor.class, "duck");

        capHand    = hwMap.get(DcMotor.class, "hand");
//
        capArm         = hwMap.get(Servo.class, "arm");

        holder         = hwMap.get(Servo.class, "holder");


        colorRight    = hwMap.get(ColorSensor.class, "colorRight");
        colorLeft     = hwMap.get(ColorSensor.class, "colorLeft");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();






        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        lifter.setDirection(DcMotorSimple.Direction.REVERSE);
        duck.setDirection(DcMotorSimple.Direction.FORWARD);
        capHand.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Set to REVERSE if using AndyMark motors
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Set to FORWARD if using AndyMark motors
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Set to REVERSE if using AndyMark motors
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);// Set to FORWARD if using AndyMark motors
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        capHand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        holder.setDirection(Servo.Direction.FORWARD);
        capArm.setDirection(Servo.Direction.REVERSE);



        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        intake.setPower(0);
        lifter.setPower(0);
        duck.setPower(0);
        capHand.setPower(0);


        holder.setPosition(0.85);


        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        capHand.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        float hsvValues[] = {0F, 0F, 0F};


        final float values[] = hsvValues;

        final double SCALE_FACTOR = 255;


        int relativeLayoutId = hwMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hwMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hwMap.appContext).findViewById(relativeLayoutId);
    }

    public void setAllPower(double p)
    {
        setMotorPower(p,p,p,p);
    }

    public void setMotorPower(double frontLeftSpeed, double frontRightSpeed, double backRightSpeed, double backLeftSpeed)
    {
        frontLeft.setPower(frontLeftSpeed);
        frontRight.setPower(frontRightSpeed);
        backRight.setPower(backRightSpeed);
        backLeft.setPower(backLeftSpeed);

    }
}

