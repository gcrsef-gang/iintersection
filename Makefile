CXX := g++
CXX_FLAGS := -pthread -Wno-unused-result -Wsign-compare -DNDEBUG -DPUGIXML_HEADER_ONLY -ggdb -fwrapv -O3 -Wall
INCLUDE := -Ilib/ -Ilib/sumo/src -Ilib/sumo/build/src/ -Ilib/sumo/build/cmake-build/src/ -I/usr/include/fox-1.6/ -I/usr/include/python3.9
LD_FLAGS := -Wl,-O1,--sort-common,--no-as-needed,-z,relro,-z,now,--no-undefined
LIBS :=  -L/usr/lib/ -lxerces-c -lz -lproj -lFOX-1.6 -lX11 -lXext -lfreetype -lfontconfig -lXft -lXcursor -lXrender -lXrandr -lXfixes -lXi -lGL -lGLU -ldl -lpthread -lrt -ljpeg -lpng -ltiff -lz -lbz2 libsumo.a -lpython3.9
SUMOLIBS := lib/sumo/build/cmake-build/src/microsim/libmicrosim.a lib/sumo/build/cmake-build/src/traci-server/libtraciserver.a lib/sumo/build/cmake-build/src/libsumo/liblibsumostatic.a lib/sumo/build/cmake-build/src/traci-server/libtraciserver.a lib/sumo/build/cmake-build/src/netload/libnetload.a lib/sumo/build/cmake-build/src/microsim/cfmodels/libmicrosim_cfmodels.a lib/sumo/build/cmake-build/src/microsim/engine/libmicrosim_engine.a lib/sumo/build/cmake-build/src/microsim/lcmodels/libmicrosim_lcmodels.a lib/sumo/build/cmake-build/src/microsim/devices/libmicrosim_devices.a lib/sumo/build/cmake-build/src/microsim/trigger/libmicrosim_trigger.a lib/sumo/build/cmake-build/src/microsim/output/libmicrosim_output.a lib/sumo/build/cmake-build/src/microsim/transportables/libmicrosim_transportables.a lib/sumo/build/cmake-build/src/microsim/actions/libmicrosim_actions.a lib/sumo/build/cmake-build/src/microsim/traffic_lights/libmicrosim_traffic_lights.a lib/sumo/build/cmake-build/src/microsim/libmicrosim.a lib/sumo/build/cmake-build/src/mesosim/libmesosim.a lib/sumo/build/cmake-build/src/utils/emissions/libutils_emissions.a lib/sumo/build/cmake-build/src/foreign/PHEMlight/cpp/libforeign_phemlight.a lib/sumo/build/cmake-build/src/utils/vehicle/libutils_vehicle.a lib/sumo/build/cmake-build/src/utils/distribution/libutils_distribution.a lib/sumo/build/cmake-build/src/utils/shapes/libutils_shapes.a lib/sumo/build/cmake-build/src/utils/options/libutils_options.a lib/sumo/build/cmake-build/src/utils/xml/libutils_xml.a lib/sumo/build/cmake-build/src/utils/geom/libutils_geom.a lib/sumo/build/cmake-build/src/utils/common/libutils_common.a lib/sumo/build/cmake-build/src/utils/importio/libutils_importio.a lib/sumo/build/cmake-build/src/utils/iodevices/libutils_iodevices.a lib/sumo/build/cmake-build/src/utils/traction_wire/libutils_traction_wire.a lib/sumo/build/cmake-build/src/foreign/tcpip/libforeign_tcpip.a


all: libiintersection.cpython-39-x86_64-linux-gnu.so

clean:
	rm libiintersection.o

libsumo.a:
	ar -rcT libsumo.a $(SUMOLIBS)

libiintersection.cpp:
	cython -3 --cplus libiintersection.pyx

libiintersection.o: libiintersection.cpp
	$(CXX) -fPIC $(CXX_FLAGS) $(INCLUDE) -c libiintersection.cpp -o libiintersection.o

libiintersection.cpython-39-x86_64-linux-gnu.so: libiintersection.o libsumo.a
	$(CXX) -shared $(CXX_FLAGS) $(LD_FLAGS) libiintersection.o  -o libiintersection.cpython-39-x86_64-linux-gnu.so $(LIBS)