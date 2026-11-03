"""
Safety:
1) Arm to home before moving the rail always
2) Nobody on the rail as well
3) Stop the farm server/RoboDk rail driver
4) E-stop in reach

Each entry in an Enum is a wrapper around the thing it holds,
and .value gives you what's inside. Here, what's inside MoveAbs is another Enum,
so you have to unwrap it at each level:

Expression	What you get
X_Axis(Enum), still wrapped
X_Axis.SinaPosInputs.value	The SinaPosInputs class, which contains Command and Status
"""

import asyncio
import random
from asyncua import Client, ua
import time
from enum import Enum
from asyncua.ua import NodeId, NodeIdType
from loguru import logger


class OPC_Interface_DB(Enum):
    # invalid
    HOME_POS = NodeId(13, 2, NodeIdType.Numeric)
    Home_Mode = NodeId(14, 2, NodeIdType.Numeric)
    Step = NodeId(15, 2, NodeIdType.Numeric)
    Start_Cycle = NodeId(16, 2, NodeIdType.Numeric)  # test cycle between 2 positions
    Custom_Home = NodeId(17, 2, NodeIdType.Numeric)  # homing

    class X_Axis(Enum):
        class SinaPosInputs(Enum):
            modePos = NodeId(36, 2, NodeIdType.Numeric)  # 2
            enableAxis = NodeId(37, 2, NodeIdType.Numeric)  # True
            cancelTraversing = NodeId(38, 2, NodeIdType.Numeric)  # e-stop
            intermediateStop = NodeId(39, 2, NodeIdType.Numeric)  # pause
            positive = NodeId(40, 2, NodeIdType.Numeric)
            negative = NodeId(41, 2, NodeIdType.Numeric)
            jog1 = NodeId(42, 2, NodeIdType.Numeric)
            jog2 = NodeId(43, 2, NodeIdType.Numeric)
            flyRef = NodeId(44, 2, NodeIdType.Numeric)
            ackError = NodeId(45, 2, NodeIdType.Numeric)
            executeMode = NodeId(46, 2, NodeIdType.Numeric)  # 0 => 1
            position = NodeId(47, 2, NodeIdType.Numeric)  # defined position in mm
            velocity = NodeId(48, 2, NodeIdType.Numeric)  # defined velocity in mm/s
            overV = NodeId(49, 2, NodeIdType.Numeric)
            overAcc = NodeId(50, 2, NodeIdType.Numeric)
            overDec = NodeId(51, 2, NodeIdType.Numeric)
            configEPos = NodeId(52, 2, NodeIdType.Numeric)
            hwIdStw = NodeId(53, 2, NodeIdType.Numeric)
            hwIdZsw = NodeId(54, 2, NodeIdType.Numeric)

        class SinaPosOutputs(Enum):
            axisEnabled = NodeId(58, 2, NodeIdType.Numeric)  # True
            axisPosOk = NodeId(59, 2, NodeIdType.Numeric)  # True
            axisSpFixed = NodeId(60, 2, NodeIdType.Numeric)
            axisRef = NodeId(61, 2, NodeIdType.Numeric)
            axisWarn = NodeId(62, 2, NodeIdType.Numeric)
            axisError = NodeId(63, 2, NodeIdType.Numeric)
            lockout = NodeId(64, 2, NodeIdType.Numeric)
            actVelocity = NodeId(65, 2, NodeIdType.Numeric)
            actPosition = NodeId(66, 2, NodeIdType.Numeric)
            actMode = NodeId(67, 2, NodeIdType.Numeric)
            ePosZsw1 = NodeId(68, 2, NodeIdType.Numeric)
            ePosZsw2 = NodeId(69, 2, NodeIdType.Numeric)
            actWarn = NodeId(70, 2, NodeIdType.Numeric)
            actFault = NodeId(71, 2, NodeIdType.Numeric)
            error = NodeId(72, 2, NodeIdType.Numeric)
            status = NodeId(73, 2, NodeIdType.Numeric)
            diagId = NodeId(74, 2, NodeIdType.Numeric)


RAIL_ROWS = {
    10: 5790.0,
    11: 6380.0,
    12: 6970.0,
    13: 7710.0,
    14: 8300.0,
    15: 8890.0,
    16: 9630.0,
    17: 10220.0,
    18: 10810.0,
    19: 11550.0,
    20: 12140.0,
    21: 12730.0,
    22: 13470.0,
    23: 14060.0,
    24: 14650.0,
    25: 15390.0,
    26: 15980.0,
    27: 16570.0,
    28: 17310.0,
    29: 17900.0,
    30: 18490.0,
    31: 19230.0,
    32: 19820.0,
    33: 20410.0,
    34: 21150.0,
    35: 21740.0,
    36: 22330.0,
    37: 23070.0,
    38: 23660.0,
    39: 24250.0,
    40: 24990.0,
    41: 25580.0,
    42: 26170.0,
    43: 26910.0,
    44: 27500.0,
    45: 28104.0,
}

MIN_MM, MAX_MM = 5790.0, 28104.0
RAIL_VELOCITY = 700  # Max = 1500.0
MOVE_TIMEOUT_S = 1000.0
TOLERANCE_MM = 1.0

PLC_URL = "opc.tcp://10.10.25.25:4840"
NAMESPACE_URI = "http://Server interface_1"

AXIS = OPC_Interface_DB.X_Axis.value
MODE_POS = AXIS.SinaPosInputs.value.modePos  # 2 always
ENABLE_AXIS = AXIS.SinaPosInputs.value.enableAxis  # True
CANCEL_TRAVERSING = AXIS.SinaPosInputs.value.cancelTraversing  # e-stop
INTERMEDIATE_STOP = AXIS.SinaPosInputs.value.intermediateStop  # pause
EXECUTE_MODE = AXIS.SinaPosInputs.value.executeMode  # 0 => 1
POSITION = AXIS.SinaPosInputs.value.position  # defined position in mm
VELOCITY = AXIS.SinaPosInputs.value.velocity  # defined velocity in mm/s

AXIS_ENABLED = AXIS.SinaPosOutputs.value.axisEnabled  # True
AXIS_POS_OK = AXIS.SinaPosOutputs.value.axisPosOk  # True
ERROR = AXIS.SinaPosOutputs.value.error
STATUS = AXIS.SinaPosOutputs.value.status
AXIS_ERROR = AXIS.SinaPosOutputs.value.axisError
ACT_POSITION = AXIS.SinaPosOutputs.value.actPosition
AXIS_POS_OK = AXIS.SinaPosOutputs.value.axisPosOk
ACK_ERROR = AXIS.SinaPosInputs.value.ackError
ACT_FAULT = AXIS.SinaPosOutputs.value.actFault


def _node(client: Client, ns: int, member):
    """The PLC node for a model entry NodeId(number, ns), using the namespace index of the live server."""

    return client.get_node(ua.NodeId(member.value.Identifier, ns))


async def _read(client, ns, member):
    return await _node(client, ns, member).read_value()


async def write_bool(client, ns, member, value: bool) -> None:
    await _node(client, ns, member).write_value(
        ua.DataValue(ua.Variant(value, ua.VariantType.Boolean))
    )


async def write_double(client, ns, member, value: float) -> None:
    await _node(client, ns, member).write_value(
        ua.DataValue(ua.Variant(value, ua.VariantType.Double))
    )


async def write_int(client, ns, member, value: int) -> None:
    await _node(client, ns, member).write_value(
        ua.DataValue(ua.Variant(value, ua.VariantType.Int16))
    )


async def write_float(client, ns, member, value: float) -> None:
    await _node(client, ns, member).write_value(
        ua.DataValue(ua.Variant(value, ua.VariantType.Float))
    )


async def wait_for(client, ns, member, expected, timeout: float) -> bool:
    """Poll a node until it equals expected. See _wait_node in the driver."""
    node = _node(client, ns, member)
    deadline = time.time() + timeout
    while time.time() < deadline:
        if await node.read_value() == expected:
            return True
        await asyncio.sleep(0.05)
    return False


async def cancel_operation(client, ns) -> None:
    await write_bool(client, ns, CANCEL_TRAVERSING, False)


async def pause(client, ns) -> None:
    await write_bool(client, ns, INTERMEDIATE_STOP, False)


async def ack_fault(client, ns) -> None:
    await write_bool(client, ns, ACK_ERROR, False)
    await write_bool(client, ns, ACK_ERROR, True)
    await asyncio.sleep(0.1)
    await write_bool(client, ns, ACK_ERROR, False)


async def prepare_axis(client, ns) -> None:
    """1) Stop if the drive reports a fault, so it shows in the log instead of being cleared
    2) Write True to ENABLE_AXIS, CANCEL_TRAVERSING and INTERMEDIATE_STOP, and 2 to MODE_POS
    3) Wait for AXIS_ENABLED, and raise RuntimeError if it stays False after 5 s
    """

    if not await wait_for(client, ns, AXIS_ERROR, False, timeout=5.0):
        raise RuntimeError(
            f"Drive fault before move, fault {await _read(client, ns, ACT_FAULT)}, "
            f"status {await _read(client, ns, STATUS)}"
        )
    
    await ack_fault(client, ns)

    await write_bool(client, ns, ENABLE_AXIS, True)
    await write_bool(client, ns, CANCEL_TRAVERSING, True)
    await write_bool(client, ns, INTERMEDIATE_STOP, True)
    await write_int(client, ns, MODE_POS, 2)

    if not await wait_for(client, ns, AXIS_ENABLED, True, timeout=5.0):
        raise RuntimeError("Axis not enabled after 5s")

    if await _read(client, ns, AXIS_ERROR):
        raise RuntimeError(f"Drive fault, status {await _read(client, ns, STATUS)}")


async def move_to(client, ns) -> None:

    rail_row = random.randint(10, 45)
    target = RAIL_ROWS[rail_row]

    await write_float(client, ns, POSITION, target)
    await write_float(client, ns, VELOCITY, RAIL_VELOCITY)

    try:
        await write_bool(client, ns, EXECUTE_MODE, False)
        await asyncio.sleep(0.5)

        if not await wait_for(client, ns, EXECUTE_MODE, False, timeout=5.0):
            raise RuntimeError(
                "PLC did not accept the move command, EXECUTE_MODE still True after 5s"
            )

        await write_bool(client, ns, EXECUTE_MODE, True)
        deadline = time.monotonic() + MOVE_TIMEOUT_S

        while time.monotonic() < deadline:
            position = await _node(client, ns, ACT_POSITION).read_value()

            if abs(position - target) <= TOLERANCE_MM and await _read(client, ns, AXIS_POS_OK):
                logger.info(f"Reached target {target:.1f} mm, actual {position:.1f} mm")
                return

            error = await _node(client, ns, ERROR).read_value()
            if error:
                status = await _node(client, ns, STATUS).read_value()
                raise RuntimeError(f"PLC rejected the move, error ID {error}, status {status}")

            if await _read(client, ns, AXIS_ERROR):
                raise RuntimeError(f"Drive fault, status {await _read(client, ns, STATUS)}")

            await asyncio.sleep(0.05)

        raise TimeoutError(f"No DONE after {MOVE_TIMEOUT_S} s")

    except BaseException:
        await cancel_operation(client, ns)
        raise

    finally:
        await write_bool(client, ns, ENABLE_AXIS, False)
        await write_bool(client, ns, EXECUTE_MODE, False)


async def main() -> None:
    while True:
        await asyncio.sleep(random.uniform(1.0, 5.0))

        async with Client(url=PLC_URL) as client:
            ns = await client.get_namespace_index(NAMESPACE_URI)

            await prepare_axis(client, ns)
            await move_to(client, ns)

            final = await _node(client, ns, ACT_POSITION).read_value()

            for row, pos in RAIL_ROWS.items():
                if final + TOLERANCE_MM >= pos >= final - TOLERANCE_MM:
                    logger.info(f"Final position {final:.1f} mm, row {row}")
                    break
            else:
                raise RuntimeError(f"Final position {final:.1f} is not within target mm")


if __name__ == "__main__":
    asyncio.run(main())
