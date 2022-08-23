package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.arcrobotics.ftclib.geometry.Transform2d
import com.arcrobotics.ftclib.geometry.Translation2d
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.spartronics4915.lib.T265Camera

@TeleOp(name = "Test T265", group = "Iterative Opmode")
class TestCameraOpMode : OpMode() {
    private val dashboard: FtcDashboard = FtcDashboard.getInstance()
    override fun init() {
        if (slamra == null) {
            slamra = T265Camera(Transform2d(), 0.1, hardwareMap.appContext)
        }
    }

    override fun init_loop() {}
    override fun start() {
        slamra.start()
    }

    override fun loop() {
        val robotRadius = 9 // inches
        val packet = TelemetryPacket()
        val field: Canvas = packet.fieldOverlay()
        val up: T265Camera.CameraUpdate = slamra.getLastReceivedCameraUpdate()
            ?: return

        // We divide by 0.0254 to convert meters to inches
        val translation = Translation2d(
            up.pose.getTranslation().getX() / 0.0254,
            up.pose.getTranslation().getY() / 0.0254
        )
        val rotation: Rotation2d = up.pose.getRotation()
        field.strokeCircle(translation.getX(), translation.getY(), robotRadius)
        val arrowX: Double = rotation.getCos() * robotRadius
        val arrowY: Double = rotation.getSin() * robotRadius
        val x1: Double = translation.getX() + arrowX / 2
        val y1: Double = translation.getY() + arrowY / 2
        val x2: Double = translation.getX() + arrowX
        val y2: Double = translation.getY() + arrowY
        field.strokeLine(x1, y1, x2, y2)
        dashboard.sendTelemetryPacket(packet)
    }

    override fun stop() {
        slamra.stop()
    }

    companion object {
        // We treat this like a singleton because there should only ever be one object per camera
        private var slamra: T265Camera? = null
    }
}