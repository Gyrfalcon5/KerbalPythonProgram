import krpc, math, time
from operator import sub
from operator import truediv

# Calculates orbital velocity using the Vis-Viva equation
def vis_viva(radius, semimajor_axis, mu):
    return math.sqrt(mu*((2./radius)-(1./semimajor_axis)))

# Calculates circularization at apoapsis
def apo_circ(conn):
    
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    vessel = conn.space_center.active_vessel

    # Plan burn using Vis-Viva
    r = vessel.orbit.apoapsis
    a_1 = vessel.orbit.semi_major_axis
    a_2 = r

    v_1 = vis_viva(r, a_1, vessel.orbit.body.gravitational_parameter)
    v_2 = vis_viva(r, a_2, vessel.orbit.body.gravitational_parameter)
    delta_v = v_2 - v_1
    node = vessel.control.add_node(ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Calculates circularization at periapsis
def peri_circ(conn):
    
    vessel = conn.space_center.active_vessel

    # Plan burn using Vis-Viva
    r = vessel.orbit.periapsis
    a_1 = vessel.orbit.semi_major_axis
    a_2 = r

    v_1 = vis_viva(r, a_1, vessel.orbit.body.gravitational_parameter)
    v_2 = vis_viva(r, a_2, vessel.orbit.body.gravitational_parameter)
    delta_v = v_2 - v_1
    node = vessel.control.add_node(conn.space_center.ut + vessel.orbit.time_to_periapsis,
                                   prograde=delta_v)
    
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
    vessel.auto_pilot.target_direction = node.burn_vector(node.reference_frame) #(0,1,0)
    vessel.auto_pilot.disengage()
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
    while ut() < (burn_start + (burn_time - 0.5)):
        pass

    vessel.control.throttle = 0.05
    print('Fine tuning')

    while remaining_burn()[1] > 0:
        pass

    vessel.control.throttle = 0
    node.remove()

    vessel.auto_pilot.reference_frame = vessel.orbital_reference_frame
    vessel.auto_pilot.target_direction = (0, 1, 0)

    vessel.auto_pilot.disengage()
    vessel.auto_pilot.engage()
    
    print('Stabilizing vessel after burn')
    # the wait function kept hanging so I made my own
    while vessel.auto_pilot.error > 1:
        time.sleep(0.1)
    print('Burn Complete!')

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
        destination_angle = start_ref_angle_burn + math.pi - target_aop - target_raan
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

    return (start_ut + delta_t) 

# Calculates the angular momentum of the current orbit of the given body
def angular_momentum(body):
    semi_latus_rectum = body.orbit.semi_major_axis * (1 - body.orbit.eccentricity ** 2.0)
    angular_momentum = math.sqrt(body.orbit.body.gravitational_parameter * semi_latus_rectum)
    return angular_momentum

# Calculates the flight path angle for the given body at the given time
def flight_angle_at_ut(conn, body, ut):
    
    ang_momentum = angular_momentum(body)

    flight_angle = None
    true_anomaly = body.orbit.true_anomaly_at_ut(ut)
    radius = body.orbit.radius_at(ut)
    speed = vis_viva(radius, body.orbit.semi_major_axis, body.orbit.body.gravitational_parameter) 
    
    ratio = ang_momentum / (radius * speed)
    if ratio > 1:
        print("Angular momentum is greater than it should be, rounding it off!")
        ratio = 1
    if true_anomaly == 0 or true_anomaly == math.pi or true_anomaly == -math.pi:
        flight_angle = 0
    elif true_anomaly > 0:
        flight_angle = math.acos(ratio)
    elif true_anomaly < 0:
        flight_angle = -math.acos(ratio)
    else:
        print("Something is wrong with the flight path angle!")

    return flight_angle

# Plans a departure burn for a hohmann transfer
def hohmann_departure_planner(conn, start, target):
    
    burn_time = hohmann_timing(conn, start, target)

    start_a = start.orbit.semi_major_axis
    start_e = start.orbit.eccentricity
    start_i = start.orbit.inclination
    start_raan = start.orbit.longitude_of_ascending_node
    start_aop = start.orbit.argument_of_periapsis

    target_a = target.orbit.semi_major_axis
    target_e = target.orbit.eccentricity
    target_i = target.orbit.inclination
    target_raan = target.orbit.longitude_of_ascending_node
    target_aop = target.orbit.argument_of_periapsis

    mu = start.orbit.body.gravitational_parameter

    start_ta_burn = start.orbit.true_anomaly_at_ut(burn_time)

    start_ref_angle_burn = start_ta_burn + start_aop + start_raan
    
    r_1 = start.orbit.radius_at(burn_time)
    destination_angle = start_ref_angle_burn + math.pi - target_aop - target_raan
    r_2 = target.orbit.radius_at_true_anomaly(destination_angle)
    transfer_a = (r_1 + r_2) / 2
    flight_angle = flight_angle_at_ut(conn, start, burn_time)

    # Get velocity before burn in the radius/theta/angular momentum frame
    velocity_pre_burn = vis_viva(r_1, start_a, mu)
    velocity_pre_burn = [velocity_pre_burn * math.cos(flight_angle),
                         velocity_pre_burn * math.sin(flight_angle)]

    # Get velocity after the burn
    velocity_post_burn = vis_viva(r_1, transfer_a, mu)

    # Calculate delta v
    delta_v = [velocity_post_burn - velocity_pre_burn[0],
               -velocity_pre_burn[1]]

    # Convert delta v to prograde/radial/normal frame
    delta_v = [delta_v[0] * math.cos(flight_angle) - delta_v[1] * math.sin(flight_angle),
               delta_v[0] * math.sin(flight_angle) + delta_v[1] * math.cos(flight_angle)]

    conn.space_center.active_vessel.control.add_node(burn_time, prograde=delta_v[0], radial=delta_v[1])

# Corrects manuever nodes to achieve a target parameter
def hohmann_corrector(conn, target_parameter, target_value, node=None,
                      directions=(False, False, False), guesses=[0, 0, 0],
                      time_delay=100, tolerance=0.1, step=1,
                      iter_limit=15):
    
    if node is None:
        node = conn.space_center.active_vessel.control.nodes[0]
    
    node.prograde = guesses[0]
    node.normal = guesses[1]
    node.radial = guesses[2]
    node.ut = conn.space_center.ut + time_delay

    num_iters = 0

    while abs(target_parameter() - target_value) > tolerance:
        
        num_iters += 1
        possible_corrections = {}
        old_dv = node.delta_v
        target_difference = abs(target_parameter() - target_value)

        def utility_check():
            try:
                delta_target = target_difference - abs(target_parameter() - target_value)
            
            except AttributeError:
                print("Step is too large and makes the target parameter not exist!")
                return None

            return delta_target 

        # Check if we are doing prograde corrections
        if directions[0]:

            # Assess the positive prograde case
            node.prograde += step
            possible_corrections['prograde+'] = utility_check()


            # Assess the negative prograde case
            node.prograde -= 2*step
            possible_corrections['prograde-'] = utility_check()
            
            # Put it back the way it was
            node.prograde += step

        # Check if we are doing normal corrections
        if directions[1]:
            
            # Assess the positive normal case
            node.normal += step
            possible_corrections['normal+'] = utility_check()


            # Assess the negative normal case
            node.normal -= 2*step
            possible_corrections['normal-'] = utility_check()
            
            # Put it back the way it was
            node.normal += step

        # Check if we are doing radial corrections
        if directions[2]:
           
            # Assess the positive radial case
            node.radial += step
            possible_corrections['radial+'] = utility_check()


            # Assess the negative radial case
            node.radial -= 2*step
            possible_corrections['radial-'] = utility_check()
            
            # Put it back the way it was
            node.radial += step
        
        if not any(value > 0 for value in possible_corrections.values()):
            step /= 2
            num_iters = 0
        elif any(value is None for value in possible_corrections.values()):
            step /= 2
            num_iters = 0
        else:
            best_utility = max(possible_corrections.values())
            best_correction = ""
            for correction, utility in possible_corrections.items():
                
                if utility == best_utility:
                    best_correction = correction
                    break

            print(best_correction)

            node.prograde += step*((best_correction == "prograde+") 
                                    - (best_correction == "prograde-"))
            node.radial += step*((best_correction == "radial+") 
                                  - (best_correction == "radial-"))
            node.normal += step*((best_correction == "normal+") 
                                  - (best_correction == "normal-"))

        if num_iters > iter_limit:
            step /= 2
            num_iters = 0
            print(step)

# Finds the angle between given vectors
def angle_between_vectors(vector_1, vector_2):

    if not len(vector_1) == len(vector_2):
        print("Vectors are not the same length!")
        return None

    sum_1 = 0
    sum_2 = 0
    dot_sum = 0
    for ind in range(len(vector_1)):
        sum_1 += vector_1[ind] ** 2
        sum_2 += vector_2[ind] ** 2
        dot_sum += vector_1[ind] * vector_2[ind]

    sum_1 = sum_1 ** 0.5
    sum_2 = sum_2 ** 0.5

    return math.acos(dot_sum / (sum_1 * sum_2))

# Finds out when you will be opposite the sun so you can make sure to land
# on the side with the sun
def daylight_opposite_time(conn):

    start_time = conn.space_center.ut
    vessel = conn.space_center.active_vessel
    orbited = vessel.orbit.body
    sun_frame = conn.space_center.bodies["Sun"].non_rotating_reference_frame
    orbit_period = vessel.orbit.period
    # We are going to assume a circular orbit to make this simpler
    vessel_pos = vessel.orbit.position_at(start_time, sun_frame)
    orbited_pos = orbited.orbit.position_at(start_time, sun_frame)
    vessel_pos = tuple(map(sub, vessel_pos, orbited_pos))
    angle_now = angle_between_vectors(orbited_pos, vessel_pos)

    vessel_pos = vessel.orbit.position_at(start_time + 100, sun_frame)
    orbited_pos = orbited.orbit.position_at(start_time + 100, sun_frame)
    vessel_pos = tuple(map(sub, vessel_pos, orbited_pos))
    angle_later = angle_between_vectors(orbited_pos, vessel_pos)

    if angle_now > angle_later:
        wait_diff = angle_now
    else:
        wait_diff = 2*math.pi - angle_now

    wait_time = orbit_period * (wait_diff / (2*math.pi))

    return start_time + wait_time

# Plans the landing burn for the function below
def landing_burn_prep(conn, safe_alt):
    
    burn_time = daylight_opposite_time(conn)

    vessel = conn.space_center.active_vessel
    
    safe_alt += vessel.orbit.body.equatorial_radius

    a_1 = vessel.orbit.semi_major_axis
    r = vessel.orbit.radius_at(burn_time)
    a_2 = (r + safe_alt) / 2.0
    v_1 = vis_viva(r, a_1, vessel.orbit.body.gravitational_parameter)
    v_2 = vis_viva(r, a_2, vessel.orbit.body.gravitational_parameter)
    delta_v = v_2 - v_1
    node = vessel.control.add_node(burn_time, prograde=delta_v)

# Performs a constant altitude burn until horizontal velocity is low
def constant_altitude_burn(conn):
    
    vessel = conn.space_center.active_vessel

    conn.space_center.warp_to(conn.space_center.ut + vessel.orbit.time_to_periapsis - 30)

    ap = vessel.auto_pilot
    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame,
        rotation = vessel.surface_reference_frame)
    
    ap.reference_frame = ref_frame
    flight = vessel.flight(ref_frame)
    ap.target_direction = tuple(map(sub, (0,0,0), flight.velocity))
    ap.engage()
    ap.wait()

    conn.space_center.warp_to(conn.space_center.ut + vessel.orbit.time_to_periapsis)
    ap.target_direction = tuple(map(sub, (0,0,0), flight.velocity))
    vessel.control.throttle = 1.0
    angle_control = 0
    while flight.horizontal_speed > 5.0:
        ap.target_direction = tuple(map(sub, (angle_control,0,0), flight.velocity))
        angle_control += flight.vertical_speed * -1
        angle_control = max([angle_control, 0])
        print(flight.velocity)
        time.sleep(0.1)

    vessel.control.throttle = 0

# Toggles landing legs
def toggle_legs(conn):
    
    vessel = conn.space_center.active_vessel
    legs = vessel.parts.legs

    for leg in legs:
        leg.deployed = not leg.deployed


# Calculates time until vessel impacts orbiting body
def impact_time(conn):

    vessel = conn.space_center.active_vessel

    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame,
        rotation = vessel.surface_reference_frame)
    
    current_flight = vessel.flight(ref_frame)
    altitude = current_flight.surface_altitude
    fall_speed = current_flight.vertical_speed
    gravity = vessel.orbit.body.surface_gravity
    
    impact_time = []

    # Set up quadratic equation
    a = 0.5 * gravity
    b = abs(fall_speed)
    c = -altitude

    impact_time.append((-b + math.sqrt(b ** 2.0 - 4 * a * c)) / (2 * a))
    impact_time.append((-b - math.sqrt(b ** 2.0 - 4 * a * c)) / (2 * a))

    if impact_time[0] > 0:
        return impact_time[0]
    else:
        return impact_time[1]

# Calculates time needed to do a hoverslam-ish landing
def hoverslam_time(conn):

    vessel = conn.space_center.active_vessel

    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame,
        rotation = vessel.surface_reference_frame)
    
    # Calculate gravitational energy here and at landing
    r_1 = vessel.orbit.radius
    r_2 = r_1 - vessel.flight().surface_altitude
    mu = vessel.orbit.body.gravitational_parameter
    grav_energy = mu * vessel.mass / r_1
    grav_energy = abs(grav_energy - mu * vessel.mass / r_2)

    # Calculate the energy from velocity
    velocity_energy = 0.5 * vessel.mass * (vessel.flight().speed)**2.0

    total_energy = grav_energy + velocity_energy

    # Convert it to velocity and use the rocket equation to figure out how
    # long to burn

    total_velocity = math.sqrt((2 / vessel.mass) * total_energy)

    force = vessel.available_thrust
    isp = vessel.specific_impulse * 9.82
    mass_0 = vessel.mass
    mass_f = mass_0 / math.exp(total_velocity/isp)
    flow_rate = force / isp
    burn_time = (mass_0 - mass_f) / flow_rate
    return burn_time



# Does a suicide burn
def suicide_burn(conn):

    vessel = conn.space_center.active_vessel

    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame,
        rotation = vessel.surface_reference_frame)
    
    ap = vessel.auto_pilot
    flight = vessel.flight(ref_frame)
    ap.target_direction = tuple(map(sub, (0,0,0), flight.velocity))
    ap.engage()

    impact = impact_time(conn)
    hoverslam = hoverslam_time(conn)*0.65 # Fudge it a bit for a better landing

    toggle_legs(conn)

    while impact_time(conn) > hoverslam:
        ap.target_direction = tuple(map(sub, (0,0,0), flight.velocity))

    vessel.control.throttle = 1

    while vessel.situation.name != "landed":
        print(vessel.situation.name)
        mean_accel = flight.speed / impact_time(conn)
        vessel.control.throttle = mean_accel / (vessel.available_thrust / vessel.mass)
        ap.target_direction = tuple(map(sub, (0,0,0), flight.velocity))
        time.sleep(0.1)

    vessel.control.throttle = 0

    ap.disengage()
    ap.sas = True

# This plans hyperbolic returns, so you get the right return altitude, ish
def return_burn(conn, return_alt):

    vessel = conn.space_center.active_vessel
    orbited_vel = vessel.orbit.body.orbit.speed
    orbited_alt = vessel.orbit.body.orbit.radius - vessel.orbit.body.sphere_of_influence 

    return_alt = vessel.orbit.body.orbit.body.equatorial_radius + return_alt
    return_a = (orbited_alt + return_alt) / 2.0
    target_vel = vis_viva(orbited_alt, return_a, vessel.orbit.body.orbit.body.gravitational_parameter)

    excess_velocity = target_vel - orbited_vel

    if excess_velocity > 0:
        # Implement this at some point maybe?
        pass
    else:
        excess_velocity = abs(excess_velocity)

        target_energy = excess_velocity ** 2.0 / 2.0
        target_sma = -vessel.orbit.body.gravitational_parameter / (2*target_energy)
        
        target_eccentricity = 1 - vessel.orbit.semi_major_axis / target_sma
        turning_angle = math.acos(-1 / target_eccentricity)
        
        ref_frame = vessel.orbit.body.orbital_reference_frame

        # Assuming that we are orbiting ccw
        burn_direction = (math.sin(turning_angle), -math.cos(turning_angle), 0)
        start_time = conn.space_center.ut
        vessel_pos = vessel.orbit.position_at(start_time, ref_frame)
        angle_now = angle_between_vectors(burn_direction, vessel_pos)
        vessel_pos = vessel.orbit.position_at(start_time + 100, ref_frame)
        angle_later = angle_between_vectors(burn_direction, vessel_pos)


        if angle_now > angle_later:
            wait_diff = angle_now
        else:
            wait_diff = 2*math.pi - angle_now
        
        
        wait_time = vessel.orbit.period * (wait_diff / (2*math.pi))
        
        radius = vessel.orbit.radius_at(start_time + wait_time)
        v_1 = vis_viva(radius, vessel.orbit.semi_major_axis, vessel.orbit.body.gravitational_parameter)
        v_2 = vis_viva(radius, target_sma, vessel.orbit.body.gravitational_parameter)


        vessel.control.add_node(wait_time + start_time, prograde=(v_2 - v_1))
        correction_parameter = lambda : vessel.control.nodes[0].orbit.next_orbit.periapsis
        hohmann_corrector(conn, correction_parameter, return_alt,
                          directions=(1,0,0), step=1.5, tolerance=10,
                          guesses=[v_2-v_1,0,0], time_delay=wait_time)

    '''
    do_node(conn)
    
    conn.space_center.warp_to(conn.space_center.ut + vessel.orbit.time_to_soi_change + 10)

    vessel.control.add_node(conn.space_center.ut)
    correction_parameter = lambda : vessel.control.nodes[0].orbit.periapsis
    hohmann_corrector(conn, correction_parameter, return_alt, directions=(0,1,0), step=0.5, tolerance=10, guesses=[0,0,0])
    do_node(conn)


    vessel.auto_pilot.disengage()
    vessel.auto_pilot.sas = True
    vessel.control.sas_mode = conn.space_center.SASMode(4)
    time.sleep(10)
    vessel.control.activate_next_stage()
    vessel.control.sas_mode = conn.space_center.SASMode(3)
    '''

# Handles the process of landing and splashdown
def landing_handler(conn):
    
    vessel = conn.space_center.active_vessel

    conn.space_center.warp_to(conn.space_center.ut + vessel.orbit.time_to_periapsis - 120)

    ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
        position=vessel.orbit.body.reference_frame,
        rotation = vessel.surface_reference_frame)

    flight = vessel.flight(ref_frame)
    while flight.mean_altitude > 70000:
        time.sleep(1)

    while flight.speed > 300:
        time.sleep(1)
        if flight.mean_altitude > 70000:
            conn.space_center.warp_to(conn.space_center.ut + vessel.orbit.time_to_periapsis - 120)

    vessel.control.activate_next_stage()

    while not vessel.recoverable:
        time.sleep(1)
    
    print("Splashdown!")
    time.sleep(2)
    vessel.recover()




