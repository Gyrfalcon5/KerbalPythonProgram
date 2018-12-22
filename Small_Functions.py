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
    
   
