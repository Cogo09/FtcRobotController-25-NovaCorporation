package org.firstinspires.ftc.teamcode.HARDWARES;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.List;

public class AUTOHARDWARE extends HARDWARECONFIG {
    Pose2d startPose = null;
    private IMU imu = null;      // Control/Expansion Hub IMU
    public MecanumDrive drive = null;

    private double headingError = 0;
    public static final double COUNTS_PER_INCH_Side = -100 * 0.50;


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double targetHeading = 0;
    private double driveSpeed = 0;
    private double turnSpeed = 0;
    private double leftfrontSpeed = 0;
    private double rightfrontSpeed = 0;
    private double rightbackSpeed = 0;
    private double leftbackSpeed = 0;
    private int leftfrontTarget = 0;
    private int rightfrontTarget = 0;
    private int backrightTarget = 0;
    private int backleftTarget = 0;
    private double armMotorTarget = 0;
    public static boolean drivefinished = true;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28.0;   // eg: Rev ultraplanetary 20:1 28 counts
    static final double DRIVE_GEAR_REDUCTION = 20.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    static final double COUNT_PER_INCH_ARM = 384.5;


    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    public static final double DRIVE_SPEED = 1.0;      // Max driving speed for better distance accuracy.
    public static final double TURN_SPEED = 0.2;     // Max turn speed to limit turn rate.
    static final double HEADING_THRESHOLD = 1.0;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable.
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable.
    public static final double ARM_EXTEND = 1.0;
    public Pose2d lastPose;


    public AUTOHARDWARE(LinearOpMode om, HardwareMap hwmap, Pose2d startPose) {
        super(om, hwmap, true);
        this.startPose = startPose;
        lastPose = startPose;

        drive = new MecanumDrive(hwmap, startPose);
    }

    //!help
    public void CLLB() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(-52, -50, Math.toRadians(220)))
                                .splineToLinearHeading(new Pose2d(10, -35, Math.toRadians(180)), 0)
                                .build(),
                        endAction()
                )
        );
    }

    public void CLLR() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(-52, 50, Math.toRadians(145)))
                                .splineToLinearHeading(new Pose2d(10, 35, Math.toRadians(180)), 1)
                                .build(),
                        endAction()
                )
        );
    }

    public void REDSPEC() {
        drivefinished = true;
        Actions.runBlocking(
                new SequentialAction(
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(-52, 50, Math.toRadians(145)))
                                        .lineToXConstantHeading(-25)
                                        .turnTo(Math.toRadians(147))
                                        .build(),
                                endAction()

                        ),
                        new SequentialAction(
                               // powersub.gunAction(List.of(() -> powersub.power()))
                        ),
                        new SleepAction(1),
                        new SequentialAction(
                                //powersub.gunAction(List.of(() -> powersub.intakeon()))
                        ),
                        new SleepAction(1),
                        new SequentialAction(
                                //powersub.gunAction(List.of(() -> powersub.intakeoff())),
                                //powersub.gunAction(List.of(() -> powersub.gunoff()))
                        ),
                        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
                        //HERE IS THE PRELOAD

                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(-52, 50, Math.toRadians(147)))
                                        .strafeToLinearHeading(new Vector2d(10, 20), Math.toRadians(90))

                                        .build(),
                                endAction()
                        ),
                        new SequentialAction(
                                //powersub.gunAction(List.of(() -> powersub.intakeonl()))
                        ),
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(10, 20, Math.toRadians(95)))
                                        .lineToYConstantHeading(47)
                                        .build(),
                                endAction()
                        ),
                        new SequentialAction(
                                //powersub.gunAction(List.of(() -> powersub.intakeoff()))
                        ),
                        //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                        //||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
                        //pickup third
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(10, 47, Math.toRadians(95)))
                                        .lineToYConstantHeading(35)
                                        .strafeToLinearHeading(new Vector2d(0, 35), Math.toRadians(94))
                                        .strafeToLinearHeading(new Vector2d(0, 56), Math.toRadians(95))
                                        .build(),
                                endAction()

                        ),
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(0, 56, Math.toRadians(95)))
                                        .lineToY(20)
                                        .strafeToLinearHeading(new Vector2d(-18, 18), Math.toRadians(145))
                                        .turnTo(Math.toRadians(140))
                                        .build(),
                                endAction()
                        ),
                        new SequentialAction(
                                //powersub.gunAction(List.of(() -> powersub.littlepower()))
                        ),
                        new SleepAction(0.5),
                        new SequentialAction(
                               // powersub.gunAction(List.of(() -> powersub.intakeon()))
                        ),
                        new SleepAction(1.5),
                        new SequentialAction(
                               // powersub.gunAction(List.of(() -> powersub.intakeoff())),
                                //powersub.gunAction(List.of(() -> powersub.gunoff()))
                        ),

                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(0,56,Math.toRadians(90)))
                                        .lineToY(25)
                                        .strafeToLinearHeading(new Vector2d(-10,25),Math.toRadians(91))
                                        .build(),
                                endAction()
                        ),
                        new SequentialAction(
                               // powersub.gunAction(List.of(()->powersub.intakeonl()))
                        ),
                        new SequentialAction(
                                drive.actionBuilder(new Pose2d(-10,25,Math.toRadians(91)))
                                        .lineToX(-30)
                                        .build(),
                                endAction()
                        ),
                        new SequentialAction(
                               // powersub.gunAction(List.of(()->powersub.intakeoff()))

                        )


                ));
    }



    //theoretical ^
    //            |

    //you can delete the next couple of funcs
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the OpMode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int) (distance * COUNTS_PER_INCH);
            leftfrontTarget = frontLeftMotor.getCurrentPosition() + moveCounts;
            rightfrontTarget = frontRightMotor.getCurrentPosition() + moveCounts;
            backrightTarget = backRightMotor.getCurrentPosition() + moveCounts;
            backleftTarget = backLeftMotor.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            frontLeftMotor.setTargetPosition(leftfrontTarget);
            frontRightMotor.setTargetPosition(rightfrontTarget);
            backRightMotor.setTargetPosition(backrightTarget);
            backLeftMotor.setTargetPosition(backleftTarget);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    (frontLeftMotor.isBusy() && frontRightMotor.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
//        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * Take separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     *
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed = turn;      // save this value as a class member so it can be used by telemetry.

        leftfrontSpeed = drive - turn;
        rightfrontSpeed = drive + turn;
        rightbackSpeed = drive + turn;
        leftbackSpeed = drive - turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftfrontSpeed), Math.abs(rightfrontSpeed));
        if (max > 1.0) {
            leftfrontSpeed /= max;
            rightfrontSpeed /= max;
            leftbackSpeed /= max;
            rightbackSpeed /= max;

        }

//        frontLeftMotor.setPower(leftfrontSpeed);
//        frontRightMotor.setPower(rightfrontSpeed);
//        backRightMotor.setPower(rightbackSpeed);
//        backLeftMotor.setPower(leftbackSpeed);
    }

    /**
     * Display the various control parameters while driving
     *
     * @param straight Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {


//        if (straight) {
//            telemetry.addData("Motion", "Drive Straight");
//            telemetry.addData("Target Pos L:R", "%7d:%7d", leftfrontTarget, rightfrontTarget);
//            telemetry.addData("Actual Pos L:R", "%7d:%7d", frontLeftMotor.getCurrentPosition(),
//                    frontRightMotor.getCurrentPosition());
//        } else {
//            telemetry.addData("Motion", "Turning");
//        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr", "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds FLW : FRW", "%5.2f : %5.2f", leftfrontSpeed, rightfrontSpeed);
        telemetry.addData("Wheel Speeds BLW : BRW", "%5.2f : %5.2f", leftbackSpeed, rightbackSpeed);
        telemetry.update();
    }

    /**
     * read the Robot heading directly from the IMU (in degrees)
     */
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public void sideWaysEncoderDrive(double speed,
                                     double inches) {//+=right //-=left
        int newFRTarget;
        int newFLTarget;
        int newBRTarget;
        int newBLTarget;
        if (opMode.opModeIsActive()) {
//            if (inches < 0) {
//                newFLTarget = frontLeftMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
//                newBLTarget = backLeftMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_Side);
//                newFRTarget = frontRightMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
//                newBRTarget = backRightMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_Side);
//                frontLeftMotor.setTargetPosition(newFLTarget);//actually backleft
//                backLeftMotor.setTargetPosition(newBLTarget);//actually frontleft
//                backRightMotor.setTargetPosition(newBRTarget);
//                frontRightMotor.setTargetPosition(newFRTarget);//
//
//            }
//            if (inches > 0) {
//                newFLTarget = frontLeftMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
//                newBLTarget = backLeftMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_Side);
//                newFRTarget = frontRightMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH_Side);
//                newBRTarget = backRightMotor.getCurrentPosition() - (int) (inches * COUNTS_PER_INCH_Side);
//                frontLeftMotor.setTargetPosition(newFLTarget);//actually backleft
//                backLeftMotor.setTargetPosition(newBLTarget);//actually frontleft
//                backRightMotor.setTargetPosition(newBRTarget);//makes go forward
//                frontRightMotor.setTargetPosition(newFRTarget);
//            }
//
//            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//            backLeftMotor.setPower(Math.abs(speed));
//            frontLeftMotor.setPower(Math.abs(speed));
//            frontRightMotor.setPower(Math.abs(speed));
//            backRightMotor.setPower(Math.abs(speed));
//            while (opMode.opModeIsActive() &&
//                    backRightMotor.isBusy()) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to", "%7d:%7d", frontLeftMotor.getCurrentPosition()
//                        , backRightMotor.getCurrentPosition());
//                telemetry.addData("Running to", "%7d:%7d", backLeftMotor.getCurrentPosition()
//                        , frontRightMotor.getCurrentPosition());
//                telemetry.addData("Currently at", "%7d:%7d",
//                        frontLeftMotor.getCurrentPosition()
//                        , backRightMotor.getCurrentPosition());
//                telemetry.addData("Currently at", "%7d:%7d",
//                        backLeftMotor.getCurrentPosition()
//                        , frontRightMotor.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            backLeftMotor.setPower(0);
//            frontRightMotor.setPower(0);
//            frontLeftMotor.setPower(0);
//            backRightMotor.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    //    public void scoresample(){
//        Actions.runBlocking(ParallelAction(
//                clawsub.setHangMIDDLE();
//
//                )
//
//        );
//    }
//    class update implements Action {
//
//        @Override
//        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//            //clawsub.update();
//            //armSub.update();
//            //telemetryPacket.put("encoder1", armSub.dualEncoder.getmost());
//            //return !armSub.isUpAtTarget(150) || drivefinished;
//
//        }
//    }

//    Action Update() {
//        return new update();
//
//    }

    class end implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            drivefinished = false;
            lastPose = drive.localizer.getPose();
            return false;
        }
    }

    Action endAction() {
        return new end();
    }


//    public void armextend(double maxDriveSpeed,
//                          double distance, double heading) {
//
//        // Ensure that the OpMode is still active
//        if (opMode.opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            int moveCounts = (int) (distance * COUNT_PER_INCH_ARM);
//            armMotorTarget = armMotor1.getCurrentPosition() + moveCounts;
//
//            // Set Target FIRST, then turn on RUN_TO_POSITION
//            armMotor1.setTargetPosition((int) armMotorTarget);
//
//            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
//            // Start driving straight, and then enter the control loop
//            armMotor1.setPower(1);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opMode.opModeIsActive() &&
//                    (armMotor1.isBusy())) {
//                telemetry.addData("current pose", armMotor1.getCurrentPosition());
//                telemetry.addData("targetPose", armMotorTarget);
//                telemetry.update();
//            }
//
//            // Stop all motion & Turn off RUN_TO_POSITION
//            armMotor1.setPower(0);
//            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
}

