# clean...? It deletes all the "*.o" files.
# If possile follow build steps in compiling in eclipse

# To compile, run as:
# make <target_name> E.g., make test_tx-1
# make, will automatically compile required dependency sub-targets

#============================================================================================
arduPi.o: arduPi.cpp arduPi.h
	g++ -c arduPi.cpp -o arduPi.o
	
SX1272.o: SX1272.cpp SX1272.h
	g++ -c SX1272.cpp -o SX1272.o
	
LoRa.o: LoRa.cpp LoRa.h
	g++ -c LoRa.cpp -o LoRa.o
	
MeshNetwork.o: MeshNetwork.cpp MeshNetwork.h 
	g++ -c MeshNetwork.cpp -o MeshNetwork.o
	
Coordinator.o: Coordinator.cpp MeshNetwork.h
	g++ -c Coordinator.cpp -o Coordinator.o
	
Repeater.o: Repeater.cpp MeshNetwork.h
	g++ -c Repeater.cpp -o Repeater.o
	
EndDevice.o: EndDevice.cpp MeshNetwork.h
	g++ -c EndDevice.cpp -o EndDevice.o

#============================================================================================
test-1.o: test-1.cpp MeshNetwork.h
	g++ -c -O2 -Wall -I/usr/local/include -Winline -pipe test-1.cpp -o test-1.o
# g++ -c test-1.cpp -o test-1.o

test-1: test-1.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o
	g++ -L/usr/local/lib -lrt -lpthread -lstdc++ -lm -lgeniePi -lboost_system -lboost_thread -lboost_atomic test-1.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o -o test-1
# g++ -lrt -lpthread -lstdc++ -lboost_system -lboost_thread -lboost_atomic test-1.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o -o test-1
#============================================================================================
test-2.o: test-2.cpp MeshNetwork.h
	g++ -c -O2 -Wall -I/usr/local/include -Winline -pipe test-2.cpp -o test-2.o
# g++ -c test-1.cpp -o test-1.o

test-2: test-2.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o
	g++ -L/usr/local/lib -lrt -lpthread -lstdc++ -lm -lgeniePi -lboost_system -lboost_thread -lboost_atomic test-2.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o -o test-2
# g++ -lrt -lpthread -lstdc++ -lboost_system -lboost_thread -lboost_atomic test-1.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o -o test-1

#============================================================================================
test-3.o: test-3.cpp MeshNetwork.h
	g++ -c -O2 -Wall -I/usr/local/include -Winline -pipe test-3.cpp -o test-3.o

test-3: test-3.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o
	g++ -L/usr/local/lib -lrt -lpthread -lstdc++ -lm -lgeniePi -lboost_system -lboost_thread -lboost_atomic test-3.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o -o test-3

#============================================================================================
test-1-4D.o: test-1-4D.cpp MeshNetwork.h
	g++ -c -O2 -Wall -I/usr/local/include -Winline -pipe test-1-4D.cpp -o test-1-4D.o
# g++ -c test-1-4D.cpp -o test-1-4D.o
# g++ -c -O2 -Wall -I/usr/local/include -Winline -pipe test-1-4D.cpp -o test-1-4D.o
	
test-1-4D: test-1-4D.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o
	g++ -L/usr/local/lib -lrt -lpthread -lstdc++ -lm -lgeniePi -lboost_system -lboost_thread -lboost_atomic test-1-4D.o MeshNetwork.o Coordinator.o Repeater.o EndDevice.o SX1272.o LoRa.o arduPi.o -o test-1-4D

#============================================================================================	
clean:
	rm *.o
	
clean_test_files:
	@[ -f ./test-1 ] && rm test-1 || true 
	@[ -f ./test_tx-1 ] && rm test_tx-1 || true 
