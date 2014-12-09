all: pseudo_mavlink ca-flight

pseudo_mavlink: pseudo_mavlink.cpp
	g++ -I autopilot_interface/ -I autopilot_interface/mavlink/include/mavlink/v1.0 pseudo_mavlink.cpp autopilot_interface/serial_port.cpp autopilot_interface/autopilot_interface.cpp -o pseudo_mavlink -lpthread

ca-flight: main.cpp
	g++ -I autopilot_interface/ -I autopilot_interface/mavlink/include/mavlink/v1.0 main.cpp interpolate.cpp grid.cpp autopilot_setup.cpp autopilot_interface/serial_port.cpp autopilot_interface/autopilot_interface.cpp -o ca-flight -lpthread -lm

clean:
	 rm -rf *o pseudo_mavlink ca-flight
