

#include "autopilot_interface.h"
#include "parse_commands.cpp"

int system_id = 2;
int component_id = 50;


// ------------------------------------------------------------------------------
//   Top
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{


	// --------------------------------------------------------------------------
	//   Start Port
	// --------------------------------------------------------------------------

	char *uart_name = (char*)"/dev/ttyUSB1"; // SHOULD BE A TELEM RADIO PAIRED with TELEM2
	int baudrate = 57600;
	parse_commandline(argc, argv, uart_name, baudrate);

	Serial_Port serial_port(uart_name,baudrate);
	serial_port.start();


	// OWNSHIP POSITION AND VELOCITY, LOCAL_NED
	mavlink_local_position_ned_t lp;
	lp.time_boot_ms = (uint32_t) (get_time_usec()/1000);
	lp.x  = 0.1;
	lp.y  = 0.2;
	lp.z  = 0.3;
	lp.vx = 0.4;
	lp.vy = 0.5;
	lp.vz = 0.0;

//	mavlink_attitude_t att;
//	att.roll       = 0;
//	att.pitch      = 0;
//	att.yaw        = 0;
//	att.rollspeed  = 0;
//	att.pitchspeed = 0;
//	att.yawspeed   = 0;
//
//	mavlink_heartbeat_t hb;
//	hb.custom_mode     = 0;
//	hb.type            = 0;
//	hb.autopilot       = 0;
//	hb.base_mode       = 0;
//	hb.system_status   = 0;
//	hb.mavlink_version = 0;

	float z = 0.0;

	while (1)
	{

		mavlink_message_t lp_m, att_m, hb_m;

		lp.z = z;
		z+=0.1;

		mavlink_msg_local_position_ned_encode(system_id, component_id, &lp_m, &lp);
//		mavlink_msg_attitude_encode(system_id, component_id, &att_m, &att);
//		mavlink_msg_heartbeat_encode(system_id, component_id, &hb_m, &hb);

		serial_port.write_message(lp_m);
//		serial_port.write_message(att_m);
//		serial_port.write_message(hb_m);

		usleep(0.5e6); //2Hz

	}



	return 0;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("PSEUDO_TCAS TERMINATING AT USER REQUEST\n");
	printf("\n");

	// end program here
	exit(0);

}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main (int argc, char **argv)
{
	signal(SIGINT,quit_handler);

	// This program uses throw, wrap one big try/catch here
	try
	{
		int result = top(argc,argv);
		return result;
	}

	catch ( int error )
	{
		fprintf(stderr,"main threw exception %i \n" , error);
		return error;
	}

}


