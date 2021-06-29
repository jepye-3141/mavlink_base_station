CXX = g++
CXXFLAGS = -Wall
SOURCE = main.cpp printing.cpp mavlink_prot.cpp

main: main.o printing.o
	$(CXX) -o main $(CXXFLAGS) $(SOURCE) -lrobotcontrol

clean:
	rm -rf main main.o printing.o mavlink_prot.o