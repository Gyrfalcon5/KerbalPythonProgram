'''This file combines other function files into the complete mission'''

import krpc, time, math, Small_Functions, Launch, Hohmann

with krpc.connect(name='Mun Mission', address='Endeavor-Ubuntu.local',
                  rpc_port=50000, stream_port=50001) as conn:

    parking_orbit_altitude = 80000

    #Launch.kerbin_launch(conn, parking_orbit_altitude)

    #Hohmann.mun_transfer(conn)


    #conn.space_center.active_vessel.control.activate_next_stage()

    #Small_Functions.landing_burn_prep(conn, 4200)

    #Small_Functions.do_node(conn)

    #Small_Functions.constant_altitude_burn(conn)

    #Small_Functions.suicide_burn(conn)

    #conn.space_center.quicksave()

    conn.space_center.quickload()
