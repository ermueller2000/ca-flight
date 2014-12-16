

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "autopilot_setup.h"

#define OWNSHIP_SYSID 1
#define INTRUDER_SYSID 2
#define AUTOPILOT_ID 50

bool time_to_exit = false;
Autopilot_Interface *autopilot_interface_ref;
Serial_Port *serial_port_ref;

// ------------------------------------------------------------------------------
//   Position Get and Set Functions
// ------------------------------------------------------------------------------

void
get_position_intruder(float &x, float &y)
{
	x = autopilot_interface_ref->sorted_messages[INTRUDER_SYSID][AUTOPILOT_ID].local_position_ned.x;
	y = autopilot_interface_ref->sorted_messages[INTRUDER_SYSID][AUTOPILOT_ID].local_position_ned.y;
}

void
get_velocity_intruder(float &vx, float &vy)
{
	vx = autopilot_interface_ref->sorted_messages[INTRUDER_SYSID][AUTOPILOT_ID].local_position_ned.vx;
	vy = autopilot_interface_ref->sorted_messages[INTRUDER_SYSID][AUTOPILOT_ID].local_position_ned.vy;
}

void
get_position_ownship(float &x, float &y)
{
	x = autopilot_interface_ref->sorted_messages[OWNSHIP_SYSID][AUTOPILOT_ID].local_position_ned.x;
	y = autopilot_interface_ref->sorted_messages[OWNSHIP_SYSID][AUTOPILOT_ID].local_position_ned.y;
}

void
get_velocity_ownship(float &vx, float &vy)
{
	vx = autopilot_interface_ref->sorted_messages[OWNSHIP_SYSID][AUTOPILOT_ID].local_position_ned.vx;
	vy = autopilot_interface_ref->sorted_messages[OWNSHIP_SYSID][AUTOPILOT_ID].local_position_ned.vy;
}

void
set_velocity_ownship(float &vx, float &vy, float &yaw)
{

	// --------------------------------------------------------------------------
	//   SEND OFFBOARD COMMANDS
	// --------------------------------------------------------------------------

	// initialize command data strtuctures
	mavlink_set_position_target_local_ned_t sp;

	// Set Velocity
	set_velocity( vx       , // [m/s]
				  vy       , // [m/s]
				  0.0      , // [m/s]
				  sp        );

//	// Set Yaw
//	set_yaw( yaw, sp );  // CHECK THIS PARAM -- MC_YAWRATE_MAX

	// SEND THE COMMAND
	autopilot_interface_ref->update_setpoint(sp);

	//printf("\n");
}




// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	// autopilot interface
	try {
		autopilot_interface_ref->handle_quit(sig);
	}
	catch (int error){}
	// serial port
	try {
		serial_port_ref->handle_quit(sig);
	}
	catch (int error){}

	// end program here
	exit(0);

}
