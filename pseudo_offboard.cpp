

#include "autopilot_interface.h"
#include "parse_commands.cpp"


// ------------------------------------------------------------------------------
//   Top
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{


	// --------------------------------------------------------------------------
	//   Start Port
	// --------------------------------------------------------------------------

	char *uart_name = (char*)"/dev/ttyUSB0"; // SHOULD BE DEVELOPER CABLE to SERIAL 4
	int baudrate = 460800;
	parse_commandline(argc, argv, uart_name, baudrate);

	Serial_Port serial_port(uart_name,baudrate);
	Autopilot_Interface autopilot_interface(&serial_port);

	serial_port.start();
	autopilot_interface.start();

	int sysid;
	int compid = 50;

	Mavlink_Messages messages;
	mavlink_local_position_ned_t pos;

	while (1)
	{

		// SYSTEM 2
		sysid = 2; // BUZZER

		// copy current messages
		messages = autopilot_interface.sorted_messages[sysid][compid];

		// local position in ned frame
		pos = messages.local_position_ned;
		printf("VEHICLE %i - POSITION NED - [ % .4f % .4f % .4f ] (m)\n", sysid, pos.x, pos.y, pos.z );
		printf("VEHICLE %i - VELOCITY NED - [ % .4f % .4f % .4f ] (m)\n", sysid, pos.vx, pos.vy, pos.vz );


		// SYSTEM 3
		sysid = 3; // COCONUT

		// copy current messages
		messages = autopilot_interface.sorted_messages[sysid][compid];

		// local position in ned frame
		pos = messages.local_position_ned;
		printf("VEHICLE %i - POSITION NED - [ % .4f % .4f % .4f ] (m)\n", sysid, pos.x, pos.y, pos.z );
		printf("VEHICLE %i - VELOCITY NED - [ % .4f % .4f % .4f ] (m)\n", sysid, pos.vx, pos.vy, pos.vz );


		printf("\n");
		usleep(0.5e6); //2Hz

	}

	printf("\n");

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
	printf("PSEUDO_OFFBOARD TERMINATING AT USER REQUEST\n");
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


