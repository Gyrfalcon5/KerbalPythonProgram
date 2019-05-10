import Small_Functions, time

def mun_transfer(conn):
    
    conn.space_center.target_body = conn.space_center.bodies["Mun"]

    vessel = conn.space_center.active_vessel

    Small_Functions.hohmann_departure_planner(conn, vessel,
                                              conn.space_center.target_body)

    Small_Functions.do_node(conn)
    vessel.control.activate_next_stage()

    target_flyby = 10000
    correction_node = vessel.control.add_node(conn.space_center.ut)


    # Use the lambda function to actually pass something that updates,
    # streams don't work for some reason
    correction_parameter = lambda : correction_node.orbit.next_orbit.periapsis_altitude
    Small_Functions.hohmann_corrector(conn, correction_parameter, target_flyby,
                                      directions=(1,0,0), step=1, tolerance=10,
                                      guesses=[20,0,0], time_delay=3600)

    
    Small_Functions.do_node(conn)
    
    conn.space_center.warp_to(conn.space_center.ut 
                              + vessel.orbit.time_to_soi_change + 30)

    Small_Functions.peri_circ(conn)
    Small_Functions.do_node(conn)


def mun_return(conn):

    vessel = conn.space_center.active_vessel
    
    Small_Functions.return_burn(conn, 47000)

    Small_Functions.do_node(conn)

    conn.space_center.warp_to(conn.space_center.ut + vessel.orbit.time_to_soi_change + 10)

    vessel.control.add_node(conn.space_center.ut)
    correction_parameter = lambda : vessel.control.nodes[0].orbit.periapsis_altitude
    Small_Functions.hohmann_corrector(conn, correction_parameter, 47000,
                                      directions=(0,1,0), step=0.5,
                                      tolerance=10, guesses=[0,0,0])
    Small_Functions.do_node(conn)


    vessel.auto_pilot.disengage()
    vessel.auto_pilot.sas = True
    vessel.control.sas_mode = conn.space_center.SASMode(4)
    time.sleep(10)
    vessel.control.activate_next_stage()
    vessel.control.sas_mode = conn.space_center.SASMode(3)

    Small_Functions.landing_handler(conn)
