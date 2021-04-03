from bridge import Bridge


def main():
    config = dict(
        sharedmem_path="config/sharedmem.json",

        accel_offset_x=1.01112,
        accel_offset_y=1.00765,
        accel_offset_z=0.96534,

        gyro_offset_x=2.843,
        gyro_offset_y=-6.056,
        gyro_offset_z=-1.603,

        wheel_diameter_m=0.0545,
    )

    bridge = Bridge(**config)

    bridge.run()


if __name__ == '__main__':
    main()
