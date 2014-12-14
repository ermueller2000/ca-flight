all: ca-flight

pseudo: pseudo_mavlink pseudo_tcas pseudo_offboard

ca-flight: main.cpp
	g++ -I autopilot_interface/ -I autopilot_interface/mavlink/include/mavlink/v1.0 main.cpp interpolate.cpp grid.cpp autopilot_setup.cpp autopilot_interface/serial_port.cpp autopilot_interface/autopilot_interface.cpp -o ca-flight -lpthread -lm

pseudo_mavlink: pseudo_mavlink.cpp
	g++ -I autopilot_interface/ -I autopilot_interface/mavlink/include/mavlink/v1.0 pseudo_mavlink.cpp autopilot_interface/serial_port.cpp autopilot_interface/autopilot_interface.cpp -o pseudo_mavlink -lpthread

pseudo_tcas: pseudo_tcas.cpp
	g++ -I autopilot_interface/ -I autopilot_interface/mavlink/include/mavlink/v1.0 pseudo_tcas.cpp autopilot_interface/serial_port.cpp autopilot_interface/autopilot_interface.cpp -o pseudo_tcas -lpthread

pseudo_offboard: pseudo_offboard.cpp
	g++ -I autopilot_interface/ -I autopilot_interface/mavlink/include/mavlink/v1.0 pseudo_offboard.cpp autopilot_interface/serial_port.cpp autopilot_interface/autopilot_interface.cpp -o pseudo_offboard -lpthread

clean:
	 rm -rf *o ca-flight pseudo_mavlink pseudo_tcas pseudo_offboard
