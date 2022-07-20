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

@Autonomous(name="Red_Park", group="Jaswanth")
public class Jazzy_States_Auto_Red_Park extends LinearOpMode {

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

        testIMU();

    }

    public void testIMU()
    {

        imuStrafeRight(0.5,0,1.6,4);

    }

    public void imuStrafeRight(double speed, double targetAngle, double inches, double timeoutS)
    {
        sleep(250);
        double leftPower = speed;
        double rightPower = speed;
        int newFrontRightTarget = robot.backLeft.getCurrentPosition() - ((int)(inches * COUNTS_PER_MOTOR_REV));
        int newBackRightTarget = robot.backRight.getCurrentPosition() + ((int)(inches * COUNTS_PER_MOTOR_REV));

        robot.frontLeft.setTargetPosition(newBackRightTarget);
        robot.frontRight.setTargetPosition(newFrontRightTarget);
        robot.backLeft.setTargetPosition(newFrontRightTarget);
        robot.backRight.setTargetPosition(newBackRightTarget);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMotorPower(-leftPower,rightPower,-rightPower,leftPower);

        runtime.reset();
        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
            robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if(robot.angles.firstAngle > targetAngle)
            {
                rightPower = speed + 0.05;
                leftPower = speed - 0.05;
            }
            else if(robot.angles.firstAngle < targetAngle)
            {
                rightPower = speed - 0.05;
                leftPower = speed + 0.05;
            }
            else
            {
                rightPower = speed;
                leftPower = speed;
            }
            robot.setMotorPower(-leftPower,rightPower,-rightPower,leftPower);
            // Display it for the driver.
            telemetry.addData("Path1", "turning to ", robot.angles.firstAngle);
            telemetry.addData("Path1", "Running to %7d :%7d", newBackRightTarget, newFrontRightTarget);
            telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                    robot.frontLeft.getCurrentPosition(),
                    robot.backLeft.getCurrentPosition(),
                    robot.frontRight.getCurrentPosition(),
                    robot.backRight.getCurrentPosition());
            telemetry.update();
        }

        robot.setAllPower(0);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetMotors();
        sleep(250);
    }

    public void resetMotors()
    {
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
