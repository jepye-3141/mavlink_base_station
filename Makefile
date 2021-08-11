CXX = g++
CXXFLAGS = -Wall -Wno-narrowing
SOURCE = main.cpp printing.cpp mavlink_prot.cpp trajectory.cpp math_utils.cpp 

main: main.o printing.o
	$(CXX) -g -o main $(CXXFLAGS) $(SOURCE) -lrobotcontrol

clean:
	rm -rf main main.o printing.o mavlink_prot.o trajectory.o math_utils.o