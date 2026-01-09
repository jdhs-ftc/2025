package org.firstinspires.ftc.teamcode.mechanisms

import com.qualcomm.robotcore.hardware.DigitalChannel
import org.firstinspires.ftc.teamcode.helpers.LogTelemetry

class LaserCombo(val intake: DigitalChannel, val shooter: DigitalChannel): Mechanism {
    var balls = 0
    var intakeState = false
    var lastIntakeState = false
    var shooterState = false
    var lastShooterState = false
    var intakeRising = false
    var shooterRising = false
    var intakeFalling = false
    var shooterFalling = false

    override fun update(telemetry: LogTelemetry) {
        intakeState = intake.state
        shooterState = shooter.state
        intakeRising = false
        intakeFalling = false
        shooterRising = false
        shooterFalling = false
        if (intakeState != lastIntakeState) {
            if (intakeState) {
                intakeRising = true
            } else {
                intakeFalling = true
            }
        }
        if (shooterState != lastShooterState) {
            if (shooterState) {
                shooterRising = true
            } else {
                shooterFalling = true
            }
        }
        lastIntakeState = intakeState
        lastShooterState = shooterState

        if (intakeRising && balls < 3) {
            balls++
        }
        if (shooterFalling && balls > 0) {
            balls--
        }

        telemetry.addData("lasercombo/intakeState", intakeState)
        telemetry.addData("laserCombo/shooterState", shooterState)
        telemetry.addData("laserCombo/balls", balls)
        telemetry.addData("laserCombo/intakeRising", intakeRising)
        telemetry.addData("laserCombo/intakeFalling", intakeFalling)
        telemetry.addData("laserCombo/shooterRising", shooterRising)
        telemetry.addData("laserCombo/shooterFalling", shooterFalling)
        telemetry.addData("laserCombo/lastIntakeState", lastIntakeState)
        telemetry.addData("laserCombo/lastShooterState", lastShooterState)
    }
}