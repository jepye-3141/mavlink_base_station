CXX ?= g++
CXXFLAGS ?= -Wall -Werror -pedantic --std=c++11 -g

main.exe: main.cpp
	$(CXX) $(CXXFLAGS) main.cpp -o $@

# disable built-in rules
.SUFFIXES:

# these targets do not create any files
.PHONY: clean
clean :
	rm -vrf *.o *.exe *.gch *.dSYM *.stackdump *.out.txt