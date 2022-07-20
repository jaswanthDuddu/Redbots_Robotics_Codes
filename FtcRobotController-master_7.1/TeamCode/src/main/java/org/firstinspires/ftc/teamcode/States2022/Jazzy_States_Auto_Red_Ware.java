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

@Autonomous(name="Red_WareSide", group="Jaswanth")
public class Jazzy_States_Auto_Red_Ware extends LinearOpMode {

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
    public void testColor()
    {
        double position = checkColor();
        sleep(2000);
        telemetry.addData(">>>>Position: ", position);
        telemetry.update();

        sleep(2000);
    }
    public void testIMU()
    {
        imuDriveBackward(0.4,9.75,0,4);
        int position = checkColor();

        //level2 : -0.304
        //level3 : -0.712
        if(position == 3)
        {
            encoderArm(1,-0.712,3);
        }
        else if(position == 2)
        {
            encoderArm(1,-0.304,3);
        }
        imuStrafeRight(0.5,0,0.87,4);
        servoGate(SERVO_OPEN);
        sleep(800);
        servoGate(SERVO_CLOSE);
        imuStrafeLeft(0.5,0,0.9,4);
        if(position == 3)
        {
            encoderArm(1,0.712,3);
        }
        else if(position == 2)
        {
            encoderArm(1,0.304,3);
        }

    }

    public void resetAngle()
    {
        lastAngle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currAngle = 0;
    }
    public double getAngle()
    {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = orientation.firstAngle - lastAngle.firstAngle;
        if(deltaAngle > 180)
        {
            deltaAngle -= 360;
        }
        else if(deltaAngle < -180)
        {
            deltaAngle += 360;
        }

        currAngle += deltaAngle;
        lastAngle = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currAngle;
    }
    public void turn(double degrees)
    {
        resetAngle();
        double error = degrees;

        while(opModeIsActive() && Math.abs(error) > 13.05)
        {
            double motorPower = (error < 0 ? -0.3 : 0.3);
            robot.setMotorPower(-motorPower,motorPower,motorPower,-motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        robot.setAllPower(0);
        resetMotors();
        sleep(250);
    }

    public void turnTo(double degrees)
    {
        Orientation orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;
        if(error > 180)
        {
            error -= 360;
        }
        else if(error < -180)
        {
            error+=360;
        }
        turn(error);
    }

    public double getAbsoluteAngle()
    {
        return robot.imu.getAngularOrientation(
                AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES
        ).firstAngle;
    }
    public void turnPID(double degrees)
    {
        turnToPID(degrees + getAbsoluteAngle(), 4);
    }

    public void turnToPID(double targetAngle, double timeout)
    {
        double firstAngle = getAbsoluteAngle();
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        double motorPower = pid.update(getAbsoluteAngle());
        robot.frontLeft.setPower(-motorPower);
        robot.frontRight.setPower(motorPower);
        robot.backRight.setPower(motorPower);
        robot.backLeft.setPower(-motorPower);
        runtime.reset();

        while ((Math.abs(targetAngle - getAbsoluteAngle()) > firstAngle || pid.getLastSlope() > 0.75) && (runtime.seconds() < timeout))
        {
            motorPower = pid.update(getAbsoluteAngle());
            robot.frontLeft.setPower(-motorPower);
            robot.frontRight.setPower(motorPower);
            robot.backRight.setPower(motorPower);
            robot.backLeft.setPower(-motorPower);
            telemetry.addData("While Test", Math.abs(targetAngle - getAbsoluteAngle()));
            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
        resetMotors();
        sleep(250);
    }

    public void imuStrafeLeft(double speed, double targetAngle, double inches, double timeoutS)
    {
        sleep(250);
        double leftPower = speed;
        double rightPower = speed;
        int newFrontLeftTarget = robot.frontLeft.getCurrentPosition() - (int)(inches * COUNTS_PER_MOTOR_REV);
        int newBackLeftTarget = robot.backLeft.getCurrentPosition() + (int)(inches * COUNTS_PER_MOTOR_REV);

        robot.frontLeft.setTargetPosition(newFrontLeftTarget);
        robot.frontRight.setTargetPosition(newBackLeftTarget);
        robot.backLeft.setTargetPosition(newBackLeftTarget);
        robot.backRight.setTargetPosition(newFrontLeftTarget);

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
            telemetry.addData("Path1", "Running to %7d :%7d", newBackLeftTarget, newFrontLeftTarget);
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

    public void imuDriveBackward(double speed, double distance, double targetAngle, double timeoutS)
    {
        sleep(250);
        double leftPower = speed;
        double rightPower = speed;
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
            newLeftTarget = robot.backLeft.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
            newRightTarget = robot.frontRight.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
            newRightTarget = robot.backRight.getCurrentPosition() + (int)(-distance * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.backLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);
            robot.backRight.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setMotorPower(Math.abs(leftPower),Math.abs(rightPower),Math.abs(rightPower),Math.abs(leftPower));


            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
                robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if(robot.angles.firstAngle < targetAngle)
                {
                    rightPower = speed - 0.05;
                    leftPower = speed + 0.05;
                }
                else if(robot.angles.firstAngle > targetAngle)
                {
                    rightPower = speed + 0.05;
                    leftPower = speed - 0.05;
                }
                else
                {
                    rightPower = speed;
                    leftPower = speed;
                }

                robot.setMotorPower(Math.abs(leftPower),Math.abs(rightPower),Math.abs(rightPower),Math.abs(leftPower));

                // Display it for the driver.
                telemetry.addData("Path1", "turning to ", robot.angles.firstAngle);
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.setAllPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
        resetMotors();

    }
    public void imuDriveForward(double speed, double distance, double targetAngle, double timeoutS)
    {
        sleep(250);
        double leftPower = speed;
        double rightPower = speed;
        int newLeftTarget;
        int newRightTarget;
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newRightTarget = robot.frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newRightTarget = robot.backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            robot.frontLeft.setTargetPosition(newLeftTarget);
            robot.backLeft.setTargetPosition(newLeftTarget);
            robot.frontRight.setTargetPosition(newRightTarget);
            robot.backRight.setTargetPosition(newRightTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setMotorPower(Math.abs(leftPower),Math.abs(rightPower),Math.abs(rightPower),Math.abs(leftPower));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeft.isBusy() && robot.frontRight.isBusy() && robot.backLeft.isBusy() && robot.backRight.isBusy())) {
                robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if(robot.angles.firstAngle < targetAngle)
                {
                    rightPower = speed + 0.05;
                    leftPower = speed - 0.05;
                }
                else if(robot.angles.firstAngle > targetAngle)
                {
                    rightPower = speed - 0.05;
                    leftPower = speed + 0.05;
                }
                else
                {
                    rightPower = speed;
                    leftPower = speed;
                }

                robot.setMotorPower(Math.abs(leftPower),Math.abs(rightPower),Math.abs(rightPower),Math.abs(leftPower));

                // Display it for the driver.
                telemetry.addData("Path1", "turning to ", robot.angles.firstAngle);
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                        robot.frontLeft.getCurrentPosition(),
                        robot.backLeft.getCurrentPosition(),
                        robot.frontRight.getCurrentPosition(),
                        robot.backRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.setAllPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
        resetMotors();

    }

    public void servoGate(double count)
    {
        robot.holder.setPosition(count);
        if(count == SERVO_OPEN) {
            sleep(1200);
        }
    }
    public void encoderArm(double speed, double inches, double timeoutS)
    {
        resetMotors();
        int newTarget = robot.lifter.getCurrentPosition() + (int)((inches*10) * (COUNTS_PER_MOTOR_REV / (0.75 * 3.1415)));
        robot.lifter.setTargetPosition(newTarget);
        robot.lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        robot.lifter.setPower(Math.abs(speed*10));
        while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.lifter.isBusy())) {
            telemetry.addData("Path1",  "Running to %7d :", newTarget);
            telemetry.addData("Path2",  "Running at %7d :",
                    robot.lifter.getCurrentPosition());
            telemetry.update();
        }
        robot.lifter.setPower(0);

        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        resetMotors();
    }

    public void intake(double speed, double time)
    {
        runtime.reset();
        while (runtime.seconds()>time) {
            robot.intake.setPower(speed);
        }
    }

    public void duckC(double speed, double time)
    {
        runtime.reset();
        while (runtime.seconds()<time) {
            robot.duck.setPower(speed);
        }
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
        telemetry.addData(">>>>Level 3: ", left);//%d", robot.modernRoboticsI2cGyro.getIntegratedZValue());
        telemetry.addData(">>>>Level 2: ", right);
        telemetry.update();
        robot.colorLeft.close();
        robot.colorRight.close();

        if(left <= 62)
        {
            return 2;
        }
        else if(right <= 62)
        {
            return 1;
        }
        else
        {
            return 3;
        }
    }

    public void resetMotors()
    {
        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void turnToPIDDistance(double targetAngle, double distance, double timeout)
    {
        double firstAngle = getAbsoluteAngle();
        TurnPIDController pid = new TurnPIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);
        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = robot.frontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = robot.backLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newRightTarget = robot.frontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        newRightTarget = robot.backRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
        robot.frontLeft.setTargetPosition(-newLeftTarget);
        robot.backLeft.setTargetPosition(-newLeftTarget);
        robot.frontRight.setTargetPosition(newRightTarget);
        robot.backRight.setTargetPosition(newRightTarget);
        // Turn On RUN_TO_POSITION
        robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        robot.frontLeft.setPower(Math.abs(0.5));
        robot.backLeft.setPower(Math.abs(0.5));
        robot.frontRight.setPower(Math.abs(0.5));
        robot.backRight.setPower(Math.abs(0.5));
        // Checking lastSlope to make sure that it's not oscillating when it quits
        runtime.reset();
        while ((Math.abs(targetAngle - getAbsoluteAngle()) > firstAngle || pid.getLastSlope() > 0.75) && (runtime.seconds() < timeout))
        {
            double motorPower = pid.update(getAbsoluteAngle());

            robot.frontLeft.setPower(motorPower);
            robot.frontRight.setPower(motorPower);
            robot.backRight.setPower(motorPower);
            robot.backLeft.setPower(motorPower);

            telemetry.addData("While Test", Math.abs(targetAngle - getAbsoluteAngle()));
            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setAllPower(0);
        resetMotors();
        sleep(250);
    }
}
