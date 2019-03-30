import Small_Functions

def mun_transfer(conn):
    
    conn.space_center.target_body = conn.space_center.bodies["Mun"]

    vessel = conn.space_center.active_vessel

    Small_Functions.hohmann_departure_planner(conn, vessel,
                                              conn.space_center.target_body)

    Small_Functions.do_node(conn)

    target_flyby = 10000
    correction_node = vessel.control.add_node(conn.space_center.ut)

    # Use the lambda function to actually pass something that updates,
    # streams don't work for some reason
    correction_parameter = lambda : correction_node.orbit.next_orbit.periapsis_altitude
    Small_Functions.hohmann_corrector(conn, correction_parameter, target_flyby,
                                      directions=(1,0,0), step=1, tolerance=10,
                                      guesses=[20,0,0], time_delay=3600)

    
    vessel.control.activate_next_stage()
    Small_Functions.do_node(conn)
    
    conn.space_center.warp_to(conn.space_center.ut 
                              + vessel.orbit.time_to_soi_change + 30)

    Small_Functions.peri_circ(conn)
    Small_Functions.do_node(conn)



