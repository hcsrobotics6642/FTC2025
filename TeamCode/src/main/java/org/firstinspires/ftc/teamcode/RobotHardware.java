package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class RobotHardware {

    // Hardware
    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx intakeMotor;
    public DcMotorEx flywheelMotorLeft;

    private Servo indexerServo;
    public Servo lifterServo;
    public Servo allianceLED;
    public Servo motifLED1;
    public Servo motifLED2;
    public Servo motifLED3;
    public Servo elevationServo;

    private final LinearOpMode opMode;

    // Constants
    public static final int GPP_APRILTAG_ID = 21;
    public static final int PGP_APRILTAG_ID = 22;
    public static final int PPG_APRILTAG_ID = 23;

    public static final double SLOT_1_SHOOT_POS = 0.318;
    public static final double SLOT_2_SHOOT_POS = 0.383;
    public static final double SLOT_3_SHOOT_POS = 0.463;

    public static final double SLOT_1_INTAKE_POS = 0.21;
    public static final double SLOT_2_INTAKE_POS = 0.28;
    public static final double SLOT_3_INTAKE_POS = 0.35;

    private static final double LIFTER_LOAD_POS = 0.05;
    private static final double LIFTER_FIRE_POS = 0.52;
    private static final long LIFTER_FIRE_TIME_MS = 700;

    // Shooter
    public static final int TICKS_PER_REV_FLYWHEEL = 28;
    public static final double TARGET_RPM = 6000;

    // Intake
    private static final double INTAKE_RPM_IN = 1800;
    private static final double INTAKE_RPM_OUT = -1800;

    // Auto movement tuning
    private static final double TURN_KP = 0.018;
    private static final double TURN_MIN_POWER = 0.16;
    private static final double TURN_MAX_POWER = 0.50;
    private static final double TURN_TOLERANCE_DEG = 1.5;

    private static final double DRIVE_HEADING_KP = 0.025;
    private static final double DRIVE_HEADING_MAX_CORRECTION = 0.25;
    private static final double DRIVE_DEFAULT_TIMEOUT_SECONDS = 4.0;
    private static final double DRIVE_DISTANCE_TOLERANCE_INCHES = 0.35;

    // AUTO-AIM TUNING
    public static final double AUTO_AIM_DISTANCE_INCHES = 78.0;

    // Shooter elevation tuning
    public static final double ELEVATION_AT_PERFECT = 0.47;
    public static final double ELEVATION_AT_FAR = 0.50;
    private static final double ELEVATION_GAIN_PER_INCH = 0.00;
    private static final double ELEVATION_MIN_POS = 0.30;

    public RobotHardware(HardwareMap hardwareMap, LinearOpMode opMode) {
        this.opMode = opMode;

        // Odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // Drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right_drive");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        // Flywheel
        flywheelMotorLeft = hardwareMap.get(DcMotorEx.class, "flywheelLeft");
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Intake
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Servos
        indexerServo = hardwareMap.get(Servo.class, "indexer");
        lifterServo = hardwareMap.get(Servo.class, "lifter");
        lifterServo.setPosition(LIFTER_LOAD_POS);

        allianceLED = hardwareMap.get(Servo.class, "allianceLED");
        motifLED1 = hardwareMap.get(Servo.class, "motifLED1");
        motifLED2 = hardwareMap.get(Servo.class, "motifLED2");
        motifLED3 = hardwareMap.get(Servo.class, "motifLED3");

        // Initialize LEDs to a safe/off state.
        allianceLED.setPosition(0.0);
        motifLED1.setPosition(0.0);
        motifLED2.setPosition(0.0);
        motifLED3.setPosition(0.0);

        elevationServo = hardwareMap.get(Servo.class, "elevation");
        elevationServo.setPosition(ELEVATION_AT_FAR);
    }

    public void configurePinpoint() {
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();
    }

    public Pose2D updatePose() {
        pinpoint.update();
        return pinpoint.getPosition();
    }

    public Pose2D getPose() {
        return updatePose();
    }

    public double getXInches() {
        return getPose().getX(DistanceUnit.INCH);
    }

    public double getYInches() {
        return getPose().getY(DistanceUnit.INCH);
    }

    public double getHeadingDegrees() {
        return getPose().getHeading(AngleUnit.DEGREES);
    }

    public double getHeadingRadians() {
        return getPose().getHeading(AngleUnit.RADIANS);
    }

    /**
     * Auto-adjusts shooter elevation based on current distance to goal.
     */
    public void setElevationForDistance() {
        double distance = getDistanceToAllianceGoalInches();

        if (distance <= 0) {
            elevationServo.setPosition(ELEVATION_AT_PERFECT);
            return;
        }

        double targetDist = AUTO_AIM_DISTANCE_INCHES;

        if (Math.abs(distance - targetDist) < 3.0) {
            elevationServo.setPosition(ELEVATION_AT_PERFECT);
        } else if (distance > targetDist) {
            double extra = distance - targetDist;
            double raise = extra * ELEVATION_GAIN_PER_INCH;
            double pos = ELEVATION_AT_PERFECT + raise;
            elevationServo.setPosition(Math.min(pos, ELEVATION_AT_FAR));
        } else {
            double lower = (targetDist - distance) * 0.004;
            double pos = ELEVATION_AT_PERFECT - lower;
            elevationServo.setPosition(Math.max(pos, ELEVATION_MIN_POS));
        }
    }

    // Robot-centric drive
    public void drive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    // Field-centric drive
    public void driveFieldCentric(double forward, double strafe, double rotate) {
        pinpoint.update();
        double botHeading = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);

        double fl = rotY + rotX + rotate;
        double fr = rotY - rotX - rotate;
        double bl = rotY - rotX + rotate;
        double br = rotY + rotX - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr),
                Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    public void turnToHeading(double targetDegrees, double timeoutSeconds) {
        long startTime = System.currentTimeMillis();

        while (opMode.opModeIsActive()) {
            if (System.currentTimeMillis() - startTime > timeoutSeconds * 1000.0) {
                break;
            }

            pinpoint.update();
            double current = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
            double error = AngleUnit.DEGREES.normalize(targetDegrees - current);

            if (Math.abs(error) < TURN_TOLERANCE_DEG) {
                break;
            }

            double power = error * TURN_KP;
            power = Math.copySign(Math.max(TURN_MIN_POWER, Math.abs(power)), power);
            if (Math.abs(power) > TURN_MAX_POWER) {
                power = Math.copySign(TURN_MAX_POWER, power);
            }

            drive(0, 0, power);
            opMode.idle();
        }

        drive(0, 0, 0);
    }

    public void driveStraight(double power, double inches) {
        driveStraight(power, inches, DRIVE_DEFAULT_TIMEOUT_SECONDS);
    }

    public void driveStraight(double power, double inches, double timeoutSeconds) {
        pinpoint.update();
        Pose2D start = pinpoint.getPosition();
        double startX = start.getX(DistanceUnit.INCH);
        double startY = start.getY(DistanceUnit.INCH);
        double headingTarget = start.getHeading(AngleUnit.DEGREES);
        double targetDistance = Math.abs(inches);
        double commandedPower = Math.copySign(Math.abs(power), inches == 0 ? power : inches);
        long startTime = System.currentTimeMillis();

        while (opMode.opModeIsActive()) {
            if (System.currentTimeMillis() - startTime > timeoutSeconds * 1000.0) {
                break;
            }

            pinpoint.update();
            Pose2D currentPose = pinpoint.getPosition();
            double currentX = currentPose.getX(DistanceUnit.INCH);
            double currentY = currentPose.getY(DistanceUnit.INCH);
            double traveled = Math.hypot(currentX - startX, currentY - startY);

            if (traveled >= targetDistance - DRIVE_DISTANCE_TOLERANCE_INCHES) {
                break;
            }

            double currentHeading = currentPose.getHeading(AngleUnit.DEGREES);
            double headingError = AngleUnit.DEGREES.normalize(headingTarget - currentHeading);
            double turnCorrection = headingError * DRIVE_HEADING_KP;
            turnCorrection = Math.max(-DRIVE_HEADING_MAX_CORRECTION,
                    Math.min(DRIVE_HEADING_MAX_CORRECTION, turnCorrection));

            drive(commandedPower, 0, turnCorrection);
            opMode.idle();
        }

        drive(0, 0, 0);
    }

    // Intake – velocity controlled
    public void intakeForward() {
        double ticksPerSec = (INTAKE_RPM_IN * intakeMotor.getMotorType().getTicksPerRev()) / 60.0;
        intakeMotor.setVelocity(ticksPerSec);
    }

    public void intakeReverse() {
        double ticksPerSec = (INTAKE_RPM_OUT * intakeMotor.getMotorType().getTicksPerRev()) / 60.0;
        intakeMotor.setVelocity(ticksPerSec);
    }

    public void intakeOff() {
        intakeMotor.setPower(0);
    }

    public String getIntakeState() {
        double rpm = (intakeMotor.getVelocity() * 60.0) / intakeMotor.getMotorType().getTicksPerRev();
        if (rpm > 200) return "IN";
        if (rpm < -200) return "OUT";
        return "OFF";
    }

    // Indexer
    public void rotateToIntakeSlot(int slot) {
        switch (slot) {
            case 1:
                indexerServo.setPosition(SLOT_1_INTAKE_POS);
                break;
            case 2:
                indexerServo.setPosition(SLOT_2_INTAKE_POS);
                break;
            case 3:
                indexerServo.setPosition(SLOT_3_INTAKE_POS);
                break;
            default:
                break;
        }
    }

    public void rotateToShootSlot(int slot) {
        switch (slot) {
            case 1:
                indexerServo.setPosition(SLOT_1_SHOOT_POS);
                break;
            case 2:
                indexerServo.setPosition(SLOT_2_SHOOT_POS);
                break;
            case 3:
                indexerServo.setPosition(SLOT_3_SHOOT_POS);
                break;
            default:
                break;
        }
    }

    // Shooter – velocity PID
    public void setFlywheelRPM(double rpm) {
        double ticksPerSecond = (rpm * TICKS_PER_REV_FLYWHEEL) / 60.0;
        flywheelMotorLeft.setVelocity(ticksPerSecond);
    }

    public void stopFlywheel() {
        flywheelMotorLeft.setPower(0);
    }

    public double getFlywheelRPM() {
        return (flywheelMotorLeft.getVelocity() * 60.0) / TICKS_PER_REV_FLYWHEEL;
    }

    public void prepareShot() {
        setFlywheelRPM(TARGET_RPM);
        long stableStart = 0;
        long timeout = System.currentTimeMillis() + 3500;

        while (opMode.opModeIsActive() && System.currentTimeMillis() < timeout) {
            if (Math.abs(TARGET_RPM - getFlywheelRPM()) < 120) {
                if (stableStart == 0) {
                    stableStart = System.currentTimeMillis();
                } else if (System.currentTimeMillis() - stableStart > 200) {
                    return;
                }
            } else {
                stableStart = 0;
            }
            opMode.idle();
        }
    }

    public void shootArtifact() {
        lifterServo.setPosition(LIFTER_FIRE_POS);
        opMode.sleep(LIFTER_FIRE_TIME_MS);
        returnLifter();
    }

    public void returnLifter() {
        lifterServo.setPosition(LIFTER_LOAD_POS);
    }

    public Limelight3A getLimelight() {
        return limelight;
    }

    // Alliance-specific high goal distance (Tag 20 = Blue, 24 = Red)
    public double getDistanceToAllianceGoalInches() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return -1;

        int targetId = (RobotData.alliance == RobotData.Alliance.Blue) ? 20 : 24;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() == targetId) {
                Pose3D botPose = result.getBotpose();
                return botPose.getPosition().z * 39.3701;
            }
        }
        return -1;
    }

    public double getAngleToAllianceGoal() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;

        int targetId = (RobotData.alliance == RobotData.Alliance.Blue) ? 20 : 24;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            if (fiducial.getFiducialId() == targetId) {
                return fiducial.getTargetXDegrees();
            }
        }
        return 0;
    }

    public void setAllianceLED() {
        if (RobotData.alliance == RobotData.Alliance.Blue) {
            allianceLED.setPosition(0.611);
        } else if (RobotData.alliance == RobotData.Alliance.Red) {
            allianceLED.setPosition(0.277);
        } else {
            allianceLED.setPosition(0.0);
        }
    }

    public void setMotifLEDs() {
        double green = 0.5;
        double purple = 0.722;
        double off = 0.0;

        switch (RobotData.pattern) {
            case GPP:
                motifLED1.setPosition(green);
                motifLED2.setPosition(purple);
                motifLED3.setPosition(purple);
                break;
            case PGP:
                motifLED1.setPosition(purple);
                motifLED2.setPosition(green);
                motifLED3.setPosition(purple);
                break;
            case PPG:
                motifLED1.setPosition(purple);
                motifLED2.setPosition(purple);
                motifLED3.setPosition(green);
                break;
            default:
                motifLED1.setPosition(off);
                motifLED2.setPosition(off);
                motifLED3.setPosition(off);
                break;
        }
    }

    /**
     * Auto-aims to the alliance goal: turns to center it, then drives to AUTO_AIM_DISTANCE_INCHES.
     * Returns when done, timeout is reached, or the target tag is lost.
     */
    public void autoAimToGoal() {
        long startTime = System.currentTimeMillis();
        long timeout = startTime + 5000;

        while (opMode.opModeIsActive() && System.currentTimeMillis() < timeout) {
            double angleError = getAngleToAllianceGoal();
            double currentDist = getDistanceToAllianceGoalInches();

            if (currentDist < 0) {
                break;
            }

            if (Math.abs(angleError) > 2.0) {
                double turnPower = angleError * 0.025;
                turnPower = Math.copySign(Math.max(0.2, Math.abs(turnPower)), turnPower);
                drive(0, 0, turnPower);
                opMode.idle();
                continue;
            }

            double distError = AUTO_AIM_DISTANCE_INCHES - currentDist;
            if (Math.abs(distError) > 2.0) {
                double drivePower = distError > 0 ? 0.4 : -0.4;
                drive(drivePower, 0, 0);
            } else {
                drive(0, 0, 0);
                return;
            }

            opMode.idle();
        }

        drive(0, 0, 0);
    }

    public int[] getMotifShotOrder() {
        switch (RobotData.pattern) {
            case GPP:
                return new int[]{1, 2, 3};
            case PGP:
                return new int[]{2, 1, 3};
            case PPG:
                return new int[]{3, 2, 1};
            default:
                return new int[]{1, 2, 3};
        }
    }
}
