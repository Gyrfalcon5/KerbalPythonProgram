import krpc, math, time


# Calculates orbital velocity using the Vis-Viva equation
def vis_viva(conn, radius, semimajor_axis):
    
    vessel = conn.space_center.active_vessel
    mu = vessel.orbit.body.gravitational_parameter
    return math.sqrt(mu*((2./radius)-(1./semimajor_axis)))

# Calculates circularization at apoapsis
def apo_circ(conn):
    
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    vessel = conn.space_center.active_vessel

    # Plan burn using Vis-Viva
    r = vessel.orbit.apoapsis
    a_1 = vessel.orbit.semi_major_axis
    a_2 = r

    v_1 = vis_viva(conn, r, a_1)
    v_2 = vis_viva(conn, r, a_2)
    delta_v = v_2 - v_1
    node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)
    
# Executes maneuver nodes to within reasonable accuracy
def do_node(conn):

    # Set up telemetry streams
    vessel = conn.space_center.active_vessel
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    node = vessel.control.nodes[0]
    delta_v = node.delta_v
    remaining_burn = conn.add_stream(node.remaining_burn_vector, vessel.reference_frame)
    time_to_node = conn.add_stream(getattr, node, 'time_to')

    # Calculate burn time using the rocket equation
    # TODO add support for staging during maneuvers
    force = vessel.available_thrust
    isp = vessel.specific_impulse * 9.82
    mass_0 = vessel.mass
    mass_f = mass_0 / math.exp(delta_v/isp)
    flow_rate = force / isp
    burn_time = (mass_0 - mass_f) / flow_rate

    # Orient ship
    vessel.auto_pilot.reference_frame = node.reference_frame
    vessel.auto_pilot.target_direction = (0,1,0)
    vessel.auto_pilot.engage()
    print('Orienting ship for maneuver')
    print('The burn is estimated to take', burn_time, 'seconds to complete')
    vessel.auto_pilot.wait()
    print('Ready to execute burn')
    
    # Wait until burn start
    print('Waiting until burn')
    burn_ut = ut() + time_to_node() - (burn_time/2.)
    lead_time = 5
    conn.space_center.warp_to(burn_ut - lead_time)

    while time_to_node() - (burn_time/2.) > 0:
        pass
    
    vessel.control.throttle = 1
    print('Executing burn')

    # Physics accurate burn waiting
    burn_start = ut()
    while ut() < (burn_start + (burn_time - 0.1)):
        pass

    vessel.control.throttle = 0.05
    print('Fine tuning')

    while remaining_burn()[1] > 0:
        pass

    vessel.control.throttle = 0
    node.remove()

    vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)
    
    print('Stabilizing vessel after burn')
    vessel.auto_pilot.wait()

# Times a hohmann transfer to the currently targetted body, assuming that it
# orbits the same body as the starting location
def hohmann_timing(conn, start, target):
    
    # Establish intermediate variables to avoid excessive queries
    start_a = start.orbit.semi_major_axis
    start_e = start.orbit.eccentricity
    start_i = start.orbit.inclination
    start_raan = start.orbit.longitude_of_ascending_node
    start_aop = start.orbit.argument_of_periapsis
    start_ta = start.orbit.true_anomaly

    target_a = target.orbit.semi_major_axis
    target_e = target.orbit.eccentricity
    target_i = target.orbit.inclination
    target_raan = target.orbit.longitude_of_ascending_node
    target_aop = target.orbit.argument_of_periapsis
    target_ta = target.orbit.true_anomaly

    mu = start.orbit.body.gravitational_parameter
    start_ut = conn.space_center.ut

    # Calculate mean motion
    target_mean_motion = math.sqrt(mu / target_a ** 3.0)
    start_mean_motion = math.sqrt(mu / start_a ** 3.0)
    target_ref_angle = target_ta + target_aop + target_raan
    start_ref_angle = start_ta + start_aop + start_raan


    # Iterate until we've got a consistent value for time step
    delta_t = 0
    delta_t_old = 10
    while abs(delta_t - delta_t_old) > .1:
        
        delta_t_old = delta_t

        start_ta_burn = start.orbit.true_anomaly_at_ut(start_ut + delta_t)

        # Calculate angles in common reference frame
        start_ref_angle_burn = start_ta_burn + start_aop + start_raan

        # Find the semimajor axis and TOF of the hohmann transfer
        r_1 = start.orbit.radius_at_true_anomaly(start_ta_burn)
        destination_angle = start_ref_angle_burn + math.pi / 2.0 - target_aop - target_raan
        r_2 = target.orbit.radius_at_true_anomaly(destination_angle)
        a_guess = (r_1 + r_2) / 2
        tof_guess = math.pi * math.sqrt(a_guess ** 3.0 / mu)

        # Solve for k = 0, then find the bounding k's and select the positive
        # time step yielded
        k = ((target_ref_angle - start_ref_angle) + target_mean_motion
                      * tof_guess - math.pi) / (2 * math.pi)

        k_1 = math.ceil(k)
        k_2 = math.floor(k)

        delta_t_1 = (((target_ref_angle - start_ref_angle) + target_mean_motion 
                   * tof_guess - (1 + 2*k_1)*math.pi) / 
                   (start_mean_motion - target_mean_motion))

        delta_t_2 = (((target_ref_angle - start_ref_angle) + target_mean_motion 
                   * tof_guess - (1 + 2*k_2)*math.pi) / 
                   (start_mean_motion - target_mean_motion))
        
        delta_t = max(delta_t_1, delta_t_2)
        print(tof_guess)

    print(delta_t)
    print('End of hohmann_planner')
