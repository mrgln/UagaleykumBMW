package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Detectio.DetectionMaster;
import org.firstinspires.ftc.teamcode.Detectio.DetectionPipeLine;

import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp
public class MainRun extends LinearOpMode {
    DetectionMaster cvMaster;
    SampleTankDrive drive1;

    Trajectory traj1;

    FtcDashboard dash;
    Color spawnColor = null;

    AnalogInput duino0, duino1, duino2;

    @Override
    public void runOpMode() {
        cvMaster = new DetectionMaster(this);
        cvMaster.observe();

        duino0 = hardwareMap.get(AnalogInput.class, "duino0");
        duino1 = hardwareMap.get(AnalogInput.class, "duino1");
        duino2 = hardwareMap.get(AnalogInput.class, "duino2");

        drive1 = new SampleTankDrive(hardwareMap);
        spawns = new LinkedList<>();

        int position = 0;
        int dir = 0;

        while (opModeInInit()) {
            position = getSpawningPosition();
            dir = getDir();

            telemetry.addData("Red", DetectionPipeLine.redArea);
            telemetry.addData("Green", DetectionPipeLine.greenArea);
            telemetry.addData("Analog0", duino0.getVoltage());
            telemetry.addData("Analog1", duino1.getVoltage());
            telemetry.addData("Analog2", duino2.getVoltage());
            telemetry.addData("Filtered Position", position);
            telemetry.addData("Direction", dir);
            telemetry.update();
        }

        Pose2d temporaryStartPose;
        boolean quals = false;

        if (position == 0) {
            quals = true;
            temporaryStartPose = newPose2d(80, 3, 90);
        } else if (position == 1) {
            temporaryStartPose = newPose2d(100, 3, 90);
        } else if (position == 2) {
            quals = true;
            temporaryStartPose = newPose2d(120, 3, 90);
        } else if (position == 3) {
            quals = true;
            temporaryStartPose = newPose2d(80, -13, 90);
        } else if (position == 4) {
            temporaryStartPose = newPose2d(100, -13, 90);
        } else {
            quals = true;
            temporaryStartPose = newPose2d(120, -13, 90);
        }

        drive1.setPoseEstimate(temporaryStartPose);

        if (quals) {
            DriveConstants.MAX_VEL = 20;
            DriveConstants.MAX_ANG_VEL = 2;
            Trajectory afterStartQuals = drive1.trajectoryBuilder(temporaryStartPose)
                    .splineTo(newVector2d(temporaryStartPose.getX() * 2.54, 30), r(90))
                    .build();
            drive1.followTrajectory(afterStartQuals);

            ElapsedTime timer = new ElapsedTime();

            while (timer.time(TimeUnit.SECONDS) < 1.0) {
                getDir();
            }

            if (getDir() == -1) {
                Trajectory loop = drive1.trajectoryBuilder(afterStartQuals.end())
                        .splineTo(newVector2d(120, 100), r(90))

                        .splineTo(newVector2d(100, 120), r(180))
                        .splineTo(newVector2d(-100, 120), r(180))
                        .splineTo(newVector2d(-120, 100), r(270))
                        .splineTo(newVector2d(-120, -100), r(270))
                        .splineTo(newVector2d(-100, -120), r(0))
                        .splineTo(newVector2d(100, -120), r(0))
                        .splineTo(newVector2d(120, -100), r(90))
                        .splineTo(newVector2d(120, 100), r(90))

                        .splineTo(newVector2d(100, 120), r(180))
                        .splineTo(newVector2d(-100, 120), r(180))
                        .splineTo(newVector2d(-120, 100), r(270))
                        .splineTo(newVector2d(-120, -100), r(270))
                        .splineTo(newVector2d(-100, -120), r(0))
                        .splineTo(newVector2d(100, -120), r(0))
                        .splineTo(newVector2d(120, -100), r(90))
                        .splineTo(newVector2d(120, 100), r(90))

                        .splineTo(newVector2d(100, 120), r(180))
                        .splineTo(newVector2d(-100, 120), r(180))
                        .splineTo(newVector2d(-120, 100), r(270))
                        .splineTo(newVector2d(-120, -100), r(270))
                        .splineTo(newVector2d(-100, -120), r(0))
                        .splineTo(newVector2d(100, -120), r(0))
                        .splineTo(newVector2d(120, -100), r(90))
                        .splineTo(newVector2d(120, 0), r(90))
                        .build();
                drive1.followTrajectory(loop);
            } else {
                Pose2d oldPose = drive1.getPoseEstimate();
                Pose2d newPose = newPose2d(-50 - (150 - oldPose.getX() * 2.54), oldPose.getY(),
                        Math.toDegrees(oldPose.getHeading()));
                drive1.setPoseEstimate(newPose);

                Trajectory loop = drive1.trajectoryBuilder(newPose)
                        .splineTo(newVector2d(-120, 100), r(90))

                        .splineTo(newVector2d(-100, 120), r(0))
                        .splineTo(newVector2d(100, 120), r(0))
                        .splineTo(newVector2d(120, 100), r(270))
                        .splineTo(newVector2d(120, -100), r(270))
                        .splineTo(newVector2d(100, -120), r(180))
                        .splineTo(newVector2d(-100, -120), r(180))
                        .splineTo(newVector2d(-120, -100), r(90))
                        .splineTo(newVector2d(-120, 100), r(90))

                        .splineTo(newVector2d(-100, 120), r(0))
                        .splineTo(newVector2d(100, 120), r(0))
                        .splineTo(newVector2d(120, 100), r(270))
                        .splineTo(newVector2d(120, -100), r(270))
                        .splineTo(newVector2d(100, -120), r(180))
                        .splineTo(newVector2d(-100, -120), r(180))
                        .splineTo(newVector2d(-120, -100), r(90))
                        .splineTo(newVector2d(-120, 100), r(90))

                        .splineTo(newVector2d(-100, 120), r(0))
                        .splineTo(newVector2d(100, 120), r(0))
                        .splineTo(newVector2d(120, 100), r(270))
                        .splineTo(newVector2d(120, -100), r(270))
                        .splineTo(newVector2d(100, -120), r(180))
                        .splineTo(newVector2d(-100, -120), r(180))
                        .splineTo(newVector2d(-120, -100), r(90))
                        .splineTo(newVector2d(-120, 0), r(90))
                        .build();

                drive1.followTrajectory(loop);
            }
            return;
        }

        Trajectory afterStartRed = drive1.trajectoryBuilder(temporaryStartPose)
                .splineTo(newVector2d(130, 53), r(90))
                .splineTo(newVector2d(130, 85), r(90))
                .splineTo(newVector2d(110, 105), r(180))
                .splineTo(newVector2d(100, 105), r(180))
                .build();

        drive1.setPoseEstimate(temporaryStartPose);

        if (getColor() == Color.GREEN) {
            Trajectory afterStartGreen = drive1.trajectoryBuilder(temporaryStartPose)
                    .splineTo(newVector2d(75, 30), r(90))
                    .splineTo(newVector2d(75, 50), r(90))
                    .splineTo(newVector2d(100, 70), r(0))
                    .splineTo(newVector2d(110, 70), r(0))
                    .splineTo(newVector2d(130, 90), r(90))
                    .splineTo(newVector2d(110, 115), r(180))
                    .splineTo(newVector2d(105, 115), r(180))
                    .build();
            drive1.followTrajectory(afterStartGreen);
        } else {
            drive1.followTrajectory(afterStartRed);
        }

        if (quals) {
            if (dir == -1) {

            } else {

            }
        }

        int lastPos = 0;
        int noneCount = 0;
        for (int zoneNumber = 2; zoneNumber <= 12; zoneNumber++) {
            noneCount = 0;
            if (getColor() == Color.GREEN) {
                Trajectory firstSegment = drive1.trajectoryBuilder(newPose2d(93, 105, 180))
                        .splineTo(newVector2d(60, 87), r(190))
                        .splineTo(newVector2d(2, 87), r(135))
                        .build();
                drive1.followTrajectory(firstSegment);
                Trajectory secondSegment;

                if (getColor() == Color.RED) {
                    // secondSegment = drive1.trajectoryBuilder(firstSegment.end())
                    // .splineTo(newVector2d(-50, 135), r(180))
                    // .splineTo(newVector2d(-95, 135), r(180))
                    // .splineTo(newVector2d(-115, 120), r(270))
                    // .splineTo(newVector2d(-115, 110), r(270))
                    // .build();
                    secondSegment = drive1.trajectoryBuilder(newPose2d(2, 90, 180))
                            .splineTo(newVector2d(-20, 111), r(90))
                            .splineTo(newVector2d(-50, 142), r(180))
                            .splineTo(newVector2d(-90, 142), r(180))
                            .splineTo(newVector2d(-107, 123.25), r(270))
                            .splineTo(newVector2d(-107, 113.25), r(270))
                            .build();
                    lastPos = 0;
                } else {
                    // secondSegment = drive1.trajectoryBuilder(firstSegment.end())uk8q
                    // .splineTo(newVector2d(-5, 95), r(180))
                    // .splineTo(newVector2d(-50, 80), r(180))
                    // .splineTo(newVector2d(-75, 90), r(135))
                    // .splineTo(newVector2d(-107, 123.25), r(270))
                    // .splineTo(newVector2d(-107, 113.25), r(270))
                    // .build();
                    secondSegment = drive1.trajectoryBuilder(newPose2d(2, 90, 180))
                            .splineTo(newVector2d(-50, 90), r(180))
                            .splineTo(newVector2d(-75, 105), r(90))
                            .splineTo(newVector2d(-75, 120), r(90))
                            .splineTo(newVector2d(-90, 142), r(180))
                            .splineTo(newVector2d(-110, 123.25), r(270))
                            .splineTo(newVector2d(-110, 113.25), r(270))
                            .build();
                    lastPos = 1;
                }
                drive1.followTrajectory(secondSegment);
            } else {
                if (getColor() == Color.NONE)
                    noneCount++;

                Trajectory firstSegment = drive1.trajectoryBuilder(afterStartRed.end())
                        .splineTo(newVector2d(56, 136), r(170))
                        .splineTo(newVector2d(2, 136), r(225))
                        .build();
                drive1.followTrajectory(firstSegment);
                Trajectory secondSegment;
                if (getColor() == Color.GREEN) {
                    // secondSegment = drive1.trajectoryBuilder(firstSegment.end())
                    // .splineTo(newVector2d(-50, 80), r(180))
                    // .splineTo(newVector2d(-75, 105), r(90))
                    // .splineTo(newVector2d(-75, 120), r(90))
                    // .splineTo(newVector2d(-95, 140), r(180))
                    // .splineTo(newVector2d(-115, 120), r(270))
                    // .splineTo(newVector2d(-115, 110), r(270))
                    // .build();
                    secondSegment = drive1.trajectoryBuilder(newPose2d(2, 136, 180))
                            .splineTo(newVector2d(-20, 110), r(270))
                            .splineTo(newVector2d(-50, 80), r(180))
                            .splineTo(newVector2d(-75, 105), r(90))
                            .splineTo(newVector2d(-75, 120), r(90))
                            .splineTo(newVector2d(-90, 142), r(180))
                            .splineTo(newVector2d(-112, 123.25), r(270))
                            .splineTo(newVector2d(-112, 113.25), r(270))
                            .build();
                    // secondSegment = drive1.trajectoryBuilder(firstSegment.end())
                    // .splineTo(newVector2d(-5, 95), r(180))
                    // .splineTo(newVector2d(-50, 80), r(180))
                    // .splineTo(newVector2d(-75, 100), r(135))
                    // .splineTo(newVector2d(-107, 123.25), r(270))
                    // .splineTo(newVector2d(-107, 1 13.25), r(270))
                    // .build();
                    lastPos = 2;
                } else {
                    if (getColor() == Color.NONE)
                        noneCount++;
                    // secondSegment = drive1.trajectoryBuilder(firstSegment.end())
                    // .splineTo(newVector2d(-50, 135), r(180))
                    // .splineTo(newVector2d(-95, 135), r(180))
                    // .splineTo(newVector2d(-115, 120), r(270))
                    // .splineTo(newVector2d(-115, 110), r(270))
                    // .build();
                    secondSegment = drive1.trajectoryBuilder(newPose2d(2, 136, 180))
                            .splineTo(newVector2d(-50, 142), r(180))
                            .splineTo(newVector2d(-90, 142), r(180))
                            .splineTo(newVector2d(-107, 120), r(270))
                            .splineTo(newVector2d(-107, 110), r(270))
                            .build();
                    lastPos = 3;
                }
                drive1.followTrajectory(secondSegment);
            }
            Pose2d lastPose = drive1.getPoseEstimate(), newPose;
            if (lastPos == 0) {
                newPose = new Pose2d(lastPose.getY() - 13.25 / 2.54, -lastPose.getX() - 2 / 2.54,
                        lastPose.getHeading() - r(90));
            } else if (lastPos == 1) {
                newPose = new Pose2d(lastPose.getY() - 13.25 / 2.54, -lastPose.getX() - 5 / 2.54,
                        lastPose.getHeading() - r(90));
            } else if (lastPos == 2) {
                newPose = new Pose2d(lastPose.getY() - 13.25 / 2.54, -lastPose.getX() - 7 / 2.54,
                        lastPose.getHeading() - r(90));
            } else {
                newPose = new Pose2d(lastPose.getY() - 10 / 2.54, -lastPose.getX() - 2 / 2.54,
                        lastPose.getHeading() - r(90));
            }
            drive1.setPoseEstimate(newPose);
            if (noneCount == 2)
                break;
        }
        if (noneCount == 2) {
            if (dir == -1) {
                Trajectory loop = drive1.trajectoryBuilder(drive1.getPoseEstimate())
                        .splineTo(newVector2d(60, 120), r(180))
                        .splineTo(newVector2d(-100, 120), r(180))
                        .splineTo(newVector2d(-120, 100), r(270))
                        .splineTo(newVector2d(-120, -100), r(270))
                        .splineTo(newVector2d(-100, -120), r(0))
                        .splineTo(newVector2d(100, -120), r(0))
                        .splineTo(newVector2d(120, -100), r(90))
                        .splineTo(newVector2d(120, 100), r(90))

                        .splineTo(newVector2d(100, 120), r(180))
                        .splineTo(newVector2d(-100, 120), r(180))
                        .splineTo(newVector2d(-120, 100), r(270))
                        .splineTo(newVector2d(-120, -100), r(270))
                        .splineTo(newVector2d(-100, -120), r(0))
                        .splineTo(newVector2d(100, -120), r(0))
                        .splineTo(newVector2d(120, -100), r(90))
                        .splineTo(newVector2d(120, 100), r(90))

                        .splineTo(newVector2d(100, 120), r(180))
                        .splineTo(newVector2d(-100, 120), r(180))
                        .splineTo(newVector2d(-120, 100), r(270))
                        .splineTo(newVector2d(-120, -100), r(270))
                        .splineTo(newVector2d(-100, -120), r(0))
                        .splineTo(newVector2d(0, -120), r(0))
                        .build();
                drive1.followTrajectory(loop);
            } else {
                Pose2d oldPose = drive1.getPoseEstimate();
                Pose2d newPose = newPose2d(-50 - (150 - oldPose.getX() * 2.54), oldPose.getY(),
                        Math.toDegrees(oldPose.getHeading()));
                drive1.setPoseEstimate(newPose);

                Trajectory loop = drive1.trajectoryBuilder(newPose)
                        .splineTo(newVector2d(-60, 120), r(0))
                        .splineTo(newVector2d(100, 120), r(0))
                        .splineTo(newVector2d(120, 100), r(270))
                        .splineTo(newVector2d(120, -100), r(270))
                        .splineTo(newVector2d(100, -120), r(180))
                        .splineTo(newVector2d(-100, -120), r(180))
                        .splineTo(newVector2d(-120, -100), r(90))
                        .splineTo(newVector2d(-120, 100), r(90))

                        .splineTo(newVector2d(-100, 120), r(0))
                        .splineTo(newVector2d(100, 120), r(0))
                        .splineTo(newVector2d(120, 100), r(270))
                        .splineTo(newVector2d(120, -100), r(270))
                        .splineTo(newVector2d(100, -120), r(180))
                        .splineTo(newVector2d(-100, -120), r(180))
                        .splineTo(newVector2d(-120, -100), r(90))
                        .splineTo(newVector2d(-120, 100), r(90))

                        .splineTo(newVector2d(-100, 120), r(0))
                        .splineTo(newVector2d(100, 120), r(0))
                        .splineTo(newVector2d(120, 100), r(270))
                        .splineTo(newVector2d(120, -100), r(270))
                        .splineTo(newVector2d(100, -120), r(180))
                        .splineTo(newVector2d(0, -120), r(180))
                        // .splineTo(newVector2d(-120, -100), r(90))
                        // .splineTo(newVector2d(-120, 0), r(90))
                        .build();

                drive1.followTrajectory(loop);
            }
        }
    }

    Queue<Double> spawns;

    private int getSpawningPosition() {
        int b0 = (duino0.getVoltage() > 2.5 ? 1 : 0);
        int b1 = (duino1.getVoltage() > 2.5 ? 1 : 0);
        int b2 = (duino2.getVoltage() > 2.5 ? 1 : 0);
        double position = b0 + b1 * 2 + b2 * 4;

        spawns.add(position);
        if (spawns.size() > 1000) {
            spawns.remove();
        }

        int[] c = { 0, 0, 0, 0, 0, 0 };

        for (double spw : spawns) {
            if (spw == 0)
                c[0]++;
            if (spw == 1)
                c[1]++;
            if (spw == 2)
                c[2]++;
            if (spw == 3)
                c[3]++;
            if (spw == 4)
                c[4]++;
            if (spw == 5)
                c[5]++;
        }

        int biggestId = 0;

        for (int spwId = 0; spwId < 6; spwId++) {
            if (c[spwId] > c[biggestId])
                biggestId = spwId;
        }

        return biggestId;
    }

    int direction = 0;
    Queue<Double> angles = new LinkedList<>();

    private double getAng() {
        angles.add(DetectionPipeLine.blueLineAngle);
        if (angles.size() > 20) {
            angles.remove();
        }
        double accumAngle = 0;
        for (double angle : angles) {
            accumAngle += angle;
        }
        accumAngle /= angles.size();
        return accumAngle;
    }

    private int getDir() {
        double ang = getAng();
        if (ang == 90) {
            return 0;
        }
        if (ang > -5) {
            return -1;
        }
        return 1;
    }

    public static int minimalAreaThreshold = 3000;

    private Color getColor() {
        if (DetectionPipeLine.redArea > minimalAreaThreshold
                && DetectionPipeLine.greenArea > minimalAreaThreshold)

            return (DetectionPipeLine.greenArea > DetectionPipeLine.redArea ? Color.GREEN : Color.RED);
        if (DetectionPipeLine.greenArea > minimalAreaThreshold)
            return Color.GREEN;
        if (DetectionPipeLine.redArea > minimalAreaThreshold)
            return Color.RED;
        return Color.NONE;
    }

    public Vector2d newVector2d(double x, double y) {
        return new Vector2d(x / 2.54, y / 2.54);
    }

    public Pose2d newPose2d(double x, double y, double heading) {
        return new Pose2d(x / 2.54, y / 2.54, Math.toRadians(heading));
    }

    public double r(double degrees) {
        return Math.toRadians(degrees);
    }

    public Pose2d newPose2d() {
        return new Pose2d();
    }

    enum Color {
        GREEN,
        RED,
        NONE
    }
}
