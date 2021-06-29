CXX = g++
CXXFLAGS = -Wall
SOURCE = main.cpp printing.cpp neue_mavlink_prot.cpp

main: main.o printing.o
	$(CXX) -o main $(CXXFLAGS) $(SOURCE) -lrobotcontrol

clean:
	rm -rf main main.o printing.o neue_mavlink_prot.o