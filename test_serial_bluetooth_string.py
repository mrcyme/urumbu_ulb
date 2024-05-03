import asyncio
from bleak import BleakClient

ADDRESS = "54:32:04:87:17:46"  # Make sure this matches the address from bluetoothctl
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"

async def run(address, loop):
    try:
        async with BleakClient(address, loop=loop, timeout=10.0) as client:
            connected = await client.is_connected()
            print(f"Connected: {connected}")

            # Write a value to the characteristic
            value = bytearray([ord('f')])  # Sending 'f' as an example
            await client.write_gatt_char(CHARACTERISTIC_UUID, value)
            print("Data sent")
    except Exception as e:
        print(f"Failed to connect or send data: {e}")

loop = asyncio.get_event_loop()
loop.run_until_complete(run(ADDRESS, loop))
