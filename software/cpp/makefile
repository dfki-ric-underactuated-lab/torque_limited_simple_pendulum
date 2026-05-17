CXX		  := g++
CXX_FLAGS := -Wall -Wextra -std=c++17 -ggdb

BIN		:= bin
#SRC		:= src
SRC_PLANT		:= pendulum_plant/src
SRC_SIMULATOR	:= simulator/src
SRC_ILQR		:= ilqr/src
SRC_UTILS		:= utils/src
INCLUDE := /usr/local/include
LIB		:= lib
LIBRARIES	:= -lyaml-cpp -lpthread

EXECUTABLE1	:= compute_ilqr
EXECUTABLE2	:= test_pendulum
SRCS1 := $(SRC_ILQR)/main_ilqr.cpp $(SRC_PLANT)/pendulum_plant.cpp $(SRC_SIMULATOR)/simulator.cpp $(SRC_ILQR)/ilqr.cpp
SRCS2 := $(SRC_SIMULATOR)/test_pendulum.cpp $(SRC_PLANT)/pendulum_plant.cpp $(SRC_SIMULATOR)/simulator.cpp $(SRC_UTILS)/csv_reader.cpp
SRCS3 := $(SRC_PLANT)/pendulum_plant.cpp $(SRC_SIMULATOR)/simulator.cpp $(SRC_ILQR)/ilqr.cpp

all: $(BIN)/$(EXECUTABLE1) $(BIN)/$(EXECUTABLE2) $(BIN)/libilqr.a

$(BIN)/$(EXECUTABLE1): $(SRCS1)
	@echo " Building ilqr ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)


$(BIN)/libilqr.a: libilqr.o
	ar rcs $@ $^

libilqr.o: $(SRCS3)
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) -c $< -o $@ $(LIBRARIES)

$(BIN)/$(EXECUTABLE2): $(SRCS2)
	@echo " Building pendulum test ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

clean:
	@echo " Clearing..."
	-rm $(BIN)/*
	-rm libilqr.o
