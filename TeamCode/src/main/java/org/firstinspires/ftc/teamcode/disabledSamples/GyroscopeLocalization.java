package org.firstinspires.ftc.teamcode.disabledSamples;

import com.github.bouyio.cyancore.debugger.DebugPacket;
import com.github.bouyio.cyancore.debugger.Debuggers;
import com.github.bouyio.cyancore.debugger.Logger;
import com.github.bouyio.cyancore.geomery.Pose2D;
import com.github.bouyio.cyancore.geomery.SmartVector;
import com.github.bouyio.cyancore.localization.GyroTankOdometry;
import com.github.bouyio.cyancore.util.Distance;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

/**
 * <p>
 *     This class demonstrates localization in CyanFTC using the IMU interface.
 *     Its difference with the legacy version is that it increases accuracy with mid-cycle position
 *     updating and distance unit safety.
 * </p>
 * */
@Disabled
@TeleOp()
public class GyroscopeLocalization extends OpMode {

    // TODO: Set constants to match the ones of the robot
    final double WHEEL_RADIUS = 000000000;
    final double ENCODER_COUNT_PER_REVOLUTION = 000000000;
    final double TICKS_TO_LINEAR_DISTANCE = 2 * Math.PI * WHEEL_RADIUS / ENCODER_COUNT_PER_REVOLUTION;


    // TODO: Set the init position coordinates and heading.
    double INITIAL_POS_X = 0000000000;
    double INITIAL_POS_Y = 0000000000;
    double INITIAL_HEADING = 000000000;

    // TODO: Set the parameters of the IMU.
    // Here's how
    // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
    final IMU.Parameters imuSystemParameters = null;

    /**The localization system.*/
    GyroTankOdometry odometry;

    // The motors whose encoders we will be using.
    DcMotor leftMotor;
    DcMotor rightMotor;

    // The heading measurement device.
    IMU imu;

    /**The logging system.*/
    Logger logger;

    @Override
    public void init() {
        // Initializing hardware.
        leftMotor = hardwareMap.get(DcMotor.class, "left_motor");
        rightMotor = hardwareMap.get(DcMotor.class, "right_motor");
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(imuSystemParameters);

        // Initializing localization system.

        // Initial position coordinates of the robot with a their distance unit.
        SmartVector initialRobotPosition = new SmartVector(
                Distance.DistanceUnit.CM,
                INITIAL_POS_X,
                INITIAL_POS_Y
                );

        // Initializing the measurement provider for the localization system.
        GyroTankOdometry.GyroTankMeasurementProvider measurementProvider = new GyroTankOdometry.GyroTankMeasurementProvider(
                leftMotor::getCurrentPosition,
                rightMotor::getCurrentPosition,
                imu.getRobotYawPitchRollAngles()::getYaw,
                TICKS_TO_LINEAR_DISTANCE
        );

        // Initializing the localization system itself.
        odometry = new GyroTankOdometry(initialRobotPosition, INITIAL_HEADING, measurementProvider);

        // Initializing localization system logging;\.
        logger = Debuggers.getGlobalLogger();
        odometry.attachLogger(logger);
    }

    @Override
    public void loop() {

        // Updating the calculations of the localization system.
        // This is NOT NECESSARY since it is usually being done automatically.
        odometry.update();

        // Getting the pose of the robot.
        // This is NOT NECESSARY since it is usually being done automatically.
        Pose2D pose = odometry.getPose();

        // Updating the debug information.
        odometry.debug();

        // Retrieving and displaying the debug information.
        DebugPacket[] loggerPackets = logger.dump();

        for (DebugPacket packet : loggerPackets) {
            telemetry.addData(packet.getHeader().getIdentifier(), packet.getValue());
        }
    }
}
