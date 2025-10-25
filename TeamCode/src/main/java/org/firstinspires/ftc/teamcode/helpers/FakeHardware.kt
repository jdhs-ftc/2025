package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorController
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareDevice
import com.qualcomm.robotcore.hardware.PIDCoefficients
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

class FakeServo : Servo {
    internal var fakeDirection: Servo.Direction = Servo.Direction.FORWARD
    internal var fakePosition: Double = 0.0
    override fun getController(): ServoController? {
        return null
    }

    override fun getPortNumber(): Int {
        return 0
    }

    override fun setDirection(direction: Servo.Direction) {
        this.fakeDirection = direction
    }

    override fun getDirection(): Servo.Direction {
        return fakeDirection
    }
    override fun setPosition(position: Double) {
        this.fakePosition = position
    }

    override fun getPosition(): Double {
        return fakePosition
    }

    override fun scaleRange(min: Double, max: Double) {
        // do nothing
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer {
        return HardwareDevice.Manufacturer.Unknown
    }

    override fun getDeviceName(): String {
        return "Fake Servo"
    }

    override fun getConnectionInfo(): String {
        return "Fake Servo"
    }

    override fun getVersion(): Int {
        return 12087
    }

    override fun resetDeviceConfigurationForOpMode() {
        // do nothing
    }

    override fun close() {
        // do nothing
    }

}

class FakeCRServo: CRServo {
    override fun getController(): ServoController? {
        return null
    }

    override fun getPortNumber(): Int {
        return 0
    }

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        return
    }

    override fun getDirection(): DcMotorSimple.Direction? {
        return null
    }

    override fun setPower(power: Double) {
        return
    }

    override fun getPower(): Double {
        return 0.0
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer? {
        return null
    }

    override fun getDeviceName(): String? {
        return null
    }

    override fun getConnectionInfo(): String? {
        return null
    }

    override fun getVersion(): Int {
        return 0
    }

    override fun resetDeviceConfigurationForOpMode() {
        return
    }

    override fun close() {
        return
    }

}


class FakeDcMotorEx: DcMotorEx {
    override fun setMotorEnable() {
        return
    }

    override fun setMotorDisable() {

    }

    override fun isMotorEnabled(): Boolean {
        return false
    }

    override fun setVelocity(angularRate: Double) {

    }

    override fun setVelocity(
        angularRate: Double,
        unit: AngleUnit?
    ) {

    }

    override fun getVelocity(): Double {
return 0.0
    }

    override fun getVelocity(unit: AngleUnit?): Double {
return 0.0
    }

    override fun setPIDCoefficients(
        mode: DcMotor.RunMode?,
        pidCoefficients: PIDCoefficients?
    ) {

    }

    override fun setPIDFCoefficients(
        mode: DcMotor.RunMode?,
        pidfCoefficients: PIDFCoefficients?
    ) {
        return
    }

    override fun setVelocityPIDFCoefficients(
        p: Double,
        i: Double,
        d: Double,
        f: Double
    ) {
        return
    }

    override fun setPositionPIDFCoefficients(p: Double) {
        return
    }

    override fun getPIDCoefficients(mode: DcMotor.RunMode?): PIDCoefficients? {
        return null
    }

    override fun getPIDFCoefficients(mode: DcMotor.RunMode?): PIDFCoefficients? {
        return null
    }

    override fun setTargetPositionTolerance(tolerance: Int) {
        return
    }

    override fun getTargetPositionTolerance(): Int {
        return 0
    }

    override fun getCurrent(unit: CurrentUnit?): Double {
        return 0.0
    }

    override fun getCurrentAlert(unit: CurrentUnit?): Double {
        return 0.0
    }

    override fun setCurrentAlert(
        current: Double,
        unit: CurrentUnit?
    ) {
        return
    }

    override fun isOverCurrent(): Boolean {
        return false
    }

    override fun getMotorType(): MotorConfigurationType? {
        return null
    }

    override fun setMotorType(motorType: MotorConfigurationType?) {
        return
    }

    override fun getController(): DcMotorController? {
        return null
    }

    override fun getPortNumber(): Int {
        return 0
    }

    override fun setZeroPowerBehavior(zeroPowerBehavior: DcMotor.ZeroPowerBehavior?) {
        return
    }

    override fun getZeroPowerBehavior(): DcMotor.ZeroPowerBehavior? {
        return null
    }

    override fun setPowerFloat() {
        return
    }

    override fun getPowerFloat(): Boolean {
        return false
    }

    override fun setTargetPosition(position: Int) {
        return
    }

    override fun getTargetPosition(): Int {
        return 0
    }

    override fun isBusy(): Boolean {
        return false
    }

    override fun getCurrentPosition(): Int {
        return 0
    }

    override fun setMode(mode: DcMotor.RunMode?) {
        return
    }

    override fun getMode(): DcMotor.RunMode? {
        return null
    }

    override fun setDirection(direction: DcMotorSimple.Direction?) {
        return
    }

    override fun getDirection(): DcMotorSimple.Direction? {
        return null
    }

    override fun setPower(power: Double) {
        return
    }

    override fun getPower(): Double {
        return 0.0
    }

    override fun getManufacturer(): HardwareDevice.Manufacturer? {
        return null
    }

    override fun getDeviceName(): String? {
        return null
    }

    override fun getConnectionInfo(): String? {
        return null
    }

    override fun getVersion(): Int {
        return 0
    }

    override fun resetDeviceConfigurationForOpMode() {
        return
    }

    override fun close() {
        return
    }

}