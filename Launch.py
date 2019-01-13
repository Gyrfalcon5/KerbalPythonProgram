import krpc, time, math, Small_Functions


def kerbin_launch(conn, target_alt):

    vessel = conn.space_center.active_vessel

    # Set up streams for telemetry
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis')
    apoapsis_alt = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
    periapsis = conn.add_stream(getattr, vessel.orbit, 'periapsis')
    periapsis_alt = conn.add_stream(getattr, vessel.orbit, 'periapsis_altitude')
    available_thrust = conn.add_stream(getattr, vessel, 'available_thrust')

    # Pre-launch control setup
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1
    vessel.auto_pilot.target_pitch_and_heading(90, 90)
    vessel.auto_pilot.engage()

    # Countdown because those are fun
    print('3...')
    time.sleep(1)
    print('2...')
    time.sleep(1)
    print('1...')
    time.sleep(1)
    print('Launch!')

    # Activate the first stage
    vessel.control.activate_next_stage()
    current_thrust = available_thrust()
    turn_angle = 0

    while apoapsis_alt() < target_alt:

        # Staging check
        if current_thrust > available_thrust() or current_thrust == 0:
            
            vessel.control.activate_next_stage()
            current_thrust = available_thrust()
            print('Staging event!')

        # Gravity turn logic
        if altitude() > 0 and altitude() < 10000:
            angle = altitude() / 10000 * 45
        elif altitude() > 10000 and altitude() < 45000:
            angle = (altitude() - 10000) / 35000 * 45 + 45
        elif altitude() > 45000:
            angle = 90

        if abs(angle - turn_angle) > 0.5:

            print('Current steering error is ', vessel.auto_pilot.error)
            turn_angle = angle
            vessel.auto_pilot.target_pitch_and_heading(90-turn_angle, 90)

            
    vessel.control.throttle = 0
    print('Coasting out of atmosphere')
    while altitude() < 70000:
        time.sleep(0.1)

    # Plan and execute circularization
    Small_Functions.apo_circ(conn)
    Small_Functions.do_node(conn)
