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
