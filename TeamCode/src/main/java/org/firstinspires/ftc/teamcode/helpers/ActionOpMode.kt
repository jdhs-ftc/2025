package org.firstinspires.ftc.teamcode.helpers

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import kotlin.contracts.ExperimentalContracts
import kotlin.contracts.contract

abstract class ActionOpMode : LinearOpMode() {
    private val dash: FtcDashboard = FtcDashboard.getInstance()
    var runningActions = ArrayList<Action>()
    private val uniqueActionsQueue = ArrayList<UniqueAction>()

    init {
        UniqueActionQueue.runningUniqueActions.clear()
    }

    protected fun runBlocking(action: Action) {
        val canvas = Canvas()
        action.preview(canvas)

        var actionStillRunning = true
        while (actionStillRunning && !isStopRequested) {
            val packet = TelemetryPacket()
            packet.fieldOverlay().operations.addAll(canvas.operations)

            actionStillRunning = action.run(packet)

            dash.sendTelemetryPacket(packet)
        }
    }

    protected fun updateAsync(packet: TelemetryPacket = DefaultPacket()) {
        // update running actions
        val newActions = ArrayList<Action>()
        for (action in runningActions) {
            action.preview(packet.fieldOverlay())
            if (action.run(packet)) {
                newActions.add(action)
            }
        }
        runningActions = newActions

        // if no packet was specified, we have to create and send one ourselves
        // this feels kinda jank tbh
        if (packet is DefaultPacket) {
            dash.sendTelemetryPacket(packet)
        }
    }


    protected fun run(a: Action) {
            runningActions.add(a)
    }
    // used to differentiate whether we just made the packet or whether it was just passed
    private class DefaultPacket(drawDefaultField: Boolean = true): TelemetryPacket(drawDefaultField)
}
