"""
Example: Using STEVE with MuJoCo for hardware-in-the-loop simulation.

This example demonstrates:
1. Creating a MuJoCo scene with valve
2. Connecting STEVE hardware to MuJoCo actuator
3. Running hardware-in-the-loop simulation
4. Synchronizing simulation and hardware states
"""

import time
import numpy as np
import mujoco as mj

from pysteve import SteveClient
from pysteve.integrations.mujoco import SteveValveActuator, HardwareSyncController

# Configuration
STEVE_IP = "192.168.1.100"
NUM_STEPS = 1000


def create_simple_scene() -> str:
    """Create simple MuJoCo XML with valve."""
    xml = """
    <mujoco model="valve_test">
        <option timestep="0.002" gravity="0 0 -9.81"/>
        
        <asset>
            <texture name="grid" type="2d" builtin="checker" width="512" height="512"
                     rgb1=".1 .2 .3" rgb2=".2 .3 .4"/>
            <material name="grid" texture="grid" texrepeat="1 1" texuniform="true"
                      reflectance=".2"/>
        </asset>
        
        <worldbody>
            <light pos="0 0 3" dir="0 0 -1"/>
            <geom name="floor" type="plane" size="2 2 .1" material="grid"/>
            
            <body name="valve_base" pos="0 0 0.5">
                <geom name="base" type="cylinder" size="0.05 0.05" rgba="0.3 0.3 0.3 1"/>
                <body name="valve_handle" pos="0 0 0.1">
                    <joint name="valve_joint" type="hinge" axis="0 0 1" limited="true"
                           range="0 1.571"/>
                    <geom name="handle" type="cylinder" size="0.02 0.08" rgba="0.8 0.2 0.2 1"/>
                    <geom name="grip" type="box" size="0.1 0.02 0.02" pos="0 0 0.1"
                          rgba="0.2 0.2 0.8 1"/>
                </body>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="valve_motor" joint="valve_joint" gear="1" ctrllimited="true"
                   ctrlrange="-0.5 0.5"/>
        </actuator>
    </mujoco>
    """
    return xml


def hardware_sync_example():
    """Example: Hardware synchronization with MuJoCo."""
    print("=" * 60)
    print("Hardware-in-the-Loop Simulation Example")
    print("=" * 60)

    # Create MuJoCo model
    xml = create_simple_scene()
    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)

    # Connect to STEVE
    client = SteveClient(STEVE_IP)
    client.connect()
    client.load_preset("smooth")
    client.enable_motor()
    time.sleep(0.5)
    client.start_valve()

    # Create STEVE actuator
    actuator = SteveValveActuator(
        client=client,
        mj_model=model,
        mj_data=data,
        joint_name="valve_joint",
        sync_mode="hardware",  # Hardware drives simulation
        latency_ms=50,
    )

    actuator.connect()

    try:
        print("\nRunning hardware-synced simulation...")
        print("Hardware state will drive MuJoCo simulation")

        for step in range(NUM_STEPS):
            # Update from hardware
            actuator.update()

            # Step MuJoCo
            mj.mj_step(model, data)

            # Print status every 100 steps
            if step % 100 == 0:
                hw_state = actuator.get_hardware_state()
                sim_pos = data.qpos[actuator.joint_id]

                print(
                    f"Step {step:4d}: "
                    f"HW pos={hw_state['position_deg']:6.2f}°, "
                    f"Sim pos={np.rad2deg(sim_pos):6.2f}°, "
                    f"Torque={hw_state['torque_nm']:6.3f} N·m"
                )

            time.sleep(0.002)  # Match timestep

        print("\nSimulation complete!")

    finally:
        actuator.disconnect()
        client.disconnect()


def hybrid_mode_example():
    """Example: Hybrid mode with bidirectional sync."""
    print("\n" + "=" * 60)
    print("Hybrid Mode Example (Bidirectional Sync)")
    print("=" * 60)

    # Create MuJoCo model
    xml = create_simple_scene()
    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)

    # Connect to STEVE
    client = SteveClient(STEVE_IP)
    client.connect()
    client.load_preset("medium")
    client.enable_motor()
    time.sleep(0.5)
    client.start_valve()

    # Create actuator in hybrid mode
    actuator = SteveValveActuator(
        client=client,
        mj_model=model,
        mj_data=data,
        joint_name="valve_joint",
        sync_mode="hybrid",  # Bidirectional sync
        latency_ms=30,
    )

    actuator.connect()

    try:
        print("\nRunning hybrid simulation...")
        print("Simulation and hardware influence each other")

        for step in range(NUM_STEPS):
            # Apply control torque in simulation
            if step < 200:
                ctrl_torque = 0.1  # Open valve
            elif step < 400:
                ctrl_torque = -0.1  # Close valve
            else:
                ctrl_torque = 0.0

            data.ctrl[0] = ctrl_torque

            # Update bidirectional sync
            actuator.update()

            # Step MuJoCo
            mj.mj_step(model, data)

            if step % 100 == 0:
                hw_state = actuator.get_hardware_state()
                sim_pos = data.qpos[actuator.joint_id]

                print(
                    f"Step {step:4d}: "
                    f"Control={ctrl_torque:5.2f}, "
                    f"HW={hw_state['position_deg']:6.2f}°, "
                    f"Sim={np.rad2deg(sim_pos):6.2f}°"
                )

            time.sleep(0.002)

        print("\nHybrid simulation complete!")

    finally:
        actuator.disconnect()
        client.disconnect()


def multi_device_example():
    """Example: Multiple STEVE devices with sync controller."""
    print("\n" + "=" * 60)
    print("Multi-Device Synchronization Example")
    print("=" * 60)

    # Device IPs
    devices = ["192.168.1.100", "192.168.1.101"]

    # Create MuJoCo model with two valves
    xml = """
    <mujoco model="dual_valve">
        <option timestep="0.002"/>
        
        <worldbody>
            <body name="valve1_base" pos="-0.3 0 0.5">
                <body name="valve1_handle" pos="0 0 0.1">
                    <joint name="valve1_joint" type="hinge" axis="0 0 1" limited="true"
                           range="0 1.571"/>
                    <geom name="handle1" type="cylinder" size="0.02 0.08" rgba="0.8 0.2 0.2 1"/>
                </body>
            </body>
            
            <body name="valve2_base" pos="0.3 0 0.5">
                <body name="valve2_handle" pos="0 0 0.1">
                    <joint name="valve2_joint" type="hinge" axis="0 0 1" limited="true"
                           range="0 1.571"/>
                    <geom name="handle2" type="cylinder" size="0.02 0.08" rgba="0.2 0.8 0.2 1"/>
                </body>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="valve1_motor" joint="valve1_joint" gear="1"/>
            <motor name="valve2_motor" joint="valve2_joint" gear="1"/>
        </actuator>
    </mujoco>
    """

    model = mj.MjModel.from_xml_string(xml)
    data = mj.MjData(model)

    # Create sync controller
    controller = HardwareSyncController(model, data)

    # Add devices
    for i, device_ip in enumerate(devices):
        controller.add_device(
            device_id=f"valve{i+1}",
            device_ip=device_ip,
            joint_name=f"valve{i+1}_joint",
            sync_mode="hardware",
        )

    # Connect all
    controller.connect_all()
    controller.start_all()

    try:
        print(f"\nSynchronizing {len(devices)} STEVE devices...")

        for step in range(500):
            # Update all devices
            controller.update_all()

            # Step simulation
            mj.mj_step(model, data)

            if step % 100 == 0:
                states = controller.get_all_hardware_states()
                print(f"\nStep {step}:")
                for device_id, state in states.items():
                    print(
                        f"  {device_id}: "
                        f"pos={state['position_deg']:6.2f}°, "
                        f"vel={state['omega_rad_s']:6.3f} rad/s"
                    )

            time.sleep(0.002)

        print("\nMulti-device sync complete!")

    finally:
        controller.disconnect_all()


def main():
    """Run all examples."""
    print("\n" + "=" * 60)
    print("STEVE MuJoCo Integration Examples")
    print("=" * 60)

    try:
        # Run examples
        hardware_sync_example()
        time.sleep(2)

        hybrid_mode_example()
        time.sleep(2)

        # Uncomment if you have multiple devices
        # multi_device_example()

        print("\n" + "=" * 60)
        print("All examples completed!")
        print("=" * 60)

    except KeyboardInterrupt:
        print("\n\nExamples interrupted by user.")
    except Exception as e:
        print(f"\n\nError: {e}")
        import traceback

        traceback.print_exc()


if __name__ == "__main__":
    main()
