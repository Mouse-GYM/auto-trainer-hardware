import pytest
import time

from pyjerrycan import JerryCAN, JerryCANMsg, JerryCANCmdType, JerryCANCfgMsg, JerryCANBootloaderCmd

DESTINATION_NODE = 0x00


@pytest.fixture(scope='module')
def jc() -> JerryCAN:
    jerrycan = JerryCAN()
    jerrycan.Open()
    yield jerrycan
    jerrycan.Close()


def test_message_send(jc):
    msg = JerryCANMsg()
    msg.type = JerryCANCmdType.HEARTBEAT

    for _ in range(5):
        jc.SendMessage(msg, 0x1F)
        time.sleep(1)


def test_message_receive(jc):
    time.sleep(1.1)

    msg = jc.ReceiveMessage()
    print(type(msg))
    if msg:
        print(msg.type)
        print(msg.dst_id)


def test_heartbeat(jc):
    for _ in range(5):
        jc.Heartbeat()
        time.sleep(0.25)


def test_estop(jc):
    jc.EStop(True)
    time.sleep(0.5)
    jc.EStop(False)


def test_stepper_move(jc):
    jc.StepperMove(DESTINATION_NODE, 0, 0, 0, 0, False)


def test_servo_move(jc):
    jc.ServoMove(DESTINATION_NODE, 0, 0, 0, 0, False)


def test_servo_detach_mode(jc):
    jc.ServoDetachMode(DESTINATION_NODE, 0, True, 0)
    jc.ServoDetachMode(DESTINATION_NODE, 0, False, 0)


def test_stepper_home(jc):
    jc.StepperHome(DESTINATION_NODE, 0)


def test_cfg_read(jc):
    msg = JerryCANCfgMsg()
    msg.type = JerryCANCfgMsg.Type.SERVO
    msg.servo.motor_id = 1
    jc.CfgRead(DESTINATION_NODE, msg)

    # Wait for the next CfgResponse type to come back
    while True:
        msg = jc.ReceiveMessage()
        if msg and msg.type == JerryCANCmdType.CFG_RESPONSE and msg.dst_id == DESTINATION_NODE:
            assert msg.cfg_response.type == JerryCANCfgMsg.Type.SERVO
            break
        time.sleep(0.001)

    msg = JerryCANCfgMsg()
    msg.type = JerryCANCfgMsg.Type.STEPPER
    msg.stepper.motor_id = 1
    msg.stepper.max_position = 100
    jc.CfgRead(DESTINATION_NODE, msg)

    # Wait for the next CfgResponse type to come back
    while True:
        msg = jc.ReceiveMessage()
        if msg and msg.type == JerryCANCmdType.CFG_RESPONSE and msg.dst_id == DESTINATION_NODE:
            assert msg.cfg_response.type == JerryCANCfgMsg.Type.STEPPER
            break
        time.sleep(0.001)


def test_cfg_write(jc):
    msg = JerryCANCfgMsg()
    msg.type = JerryCANCfgMsg.Type.SERVO
    msg.servo.motor_id = 1
    msg.servo.min_position = 0
    msg.servo.mid_position = 90
    msg.servo.max_position = 180

    jc.CfgWrite(DESTINATION_NODE, msg)


def test_bootloader_version(jc):
    jc.BootloaderCommand(DESTINATION_NODE, JerryCANBootloaderCmd.VERSION)

    # Wait for the next BootloaderResponse type to come back
    while True:
        msg = jc.ReceiveMessage()
        if msg and msg.type == JerryCANCmdType.BOOTLOADER_RESPONSE and msg.dst_id == DESTINATION_NODE:
            assert msg.bootloader_response.type == JerryCANBootloaderCmd.VERSION
            print(msg.bootloader_response.version.running_major)
            print(msg.bootloader_response.version.running_minor)
            print(msg.bootloader_response.version.running_patch)
            print(msg.bootloader_response.version.slot1_major)
            print(msg.bootloader_response.version.slot1_minor)
            print(msg.bootloader_response.version.slot1_patch)
            break
        time.sleep(0.001)
