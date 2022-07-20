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
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Color", group="Jaswanth")
public class Jazzy_States_Auto_Color extends LinearOpMode {

    Jazzy_States_Hardware   robot   = new Jazzy_States_Hardware();   // Use a Pushbot's hardware

    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    final double SERVO_OPEN = 0.4;
    final double SERVO_CLOSE = 0.85;

    float hsvValues[] = {0F, 0F, 0F};
    float hsvValues2[] = {0F, 0F, 0F};

    final float values[] = hsvValues;
    final float values2[] = hsvValues2;

    final double SCALE_FACTOR = 255;

    private Orientation lastAngle = new Orientation();
    private double currAngle = 0.0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.imu.getCalibrationStatus();
        telemetry.addData("Yo Say", "\"Jaswanth is the freakin' GOAT!\" - Hasy");
        telemetry.addData("Mode", "calibrating imu...");
        telemetry.update();

        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Yo Say", "\"Jaswanth is the freakin' GOAT!\" - Hasy");
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.holder.setPosition(SERVO_CLOSE);

        while (!isStarted()) { }

        testColor();
    }
    public void testColor()
    {
        double position = checkColor();
        sleep(2000);
        telemetry.addData(">>>>Position: ", position);
        telemetry.update();

        sleep(2000);
    }
    public int checkColor()
    {
        robot.colorLeft.enableLed(true);
        robot.colorRight.enableLed(true);
        int left = robot.colorLeft.red() +robot.colorLeft.green()+robot.colorLeft.blue();
        int right = robot.colorRight.red() + robot.colorRight.green()+robot.colorRight.blue();
        Color.RGBToHSV((int) (robot.colorLeft.red() * SCALE_FACTOR),
                (int) (robot.colorLeft.green() * SCALE_FACTOR),
                (int) (robot.colorLeft.blue() * SCALE_FACTOR),
                hsvValues);
        Color.RGBToHSV((int) (robot.colorRight.red() * SCALE_FACTOR),
                (int) (robot.colorRight.green() * SCALE_FACTOR),
                (int) (robot.colorRight.blue() * SCALE_FACTOR),
                hsvValues2);
        telemetry.addData(">>>>Left: ", left);//%d", robot.modernRoboticsI2cGyro.getIntegratedZValue());
        telemetry.addData(">>>>Right: ", right);
        telemetry.update();
        robot.colorLeft.close();
        robot.colorRight.close();

        if(left >= 112)
        {
            return 2;
        }
        else if(right >= 112)
        {
            return 1;
        }
        else
        {
            return 3;
        }
    }

}
