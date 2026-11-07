from controller import Robot


robot = Robot()

timestep = int(robot.getBasicTimeStep())

gps = robot.getDevice("gps")
imu = robot.getDevice("inertial unit")
gyro = robot.getDevice("gyro")

gps.enable(timestep)
imu.enable(timestep)
gyro.enable(timestep)

step_count = 0


while robot.step(timestep) != -1:
    gps_values = gps.getValues()
    imu_values = imu.getRollPitchYaw()
    gyro_values = gyro.getValues()

    step_count += 1
    if step_count % 125 == 0:
        print(f"Step: {step_count} | Altitude: {gps_values[2]:.2f} m | Position: ({gps_values[0]:.2f}, {gps_values[1]:.2f})")
        # print(f"alt: {gps_values[2]:.2f} m | roll: {imu_values.getRollPitchYaw()[0]:.3f} | pitch: {imu_values.getRollPitchYaw()[1]:.3f} | yaw: {imu_values.getRollPitchYaw()[2]:.3f}")
        print(f"roll: {imu_values[0]:.3f} | pitch: {imu_values[1]:.3f} | yaw: {imu_values[2]:.3f}")