CXX		  := g++
CXX_FLAGS := -Wall -Wextra -std=c++17 -ggdb

BIN		:= bin
#SRC		:= src
OBJ := obj
SRC_PLANT		:= ../../model/src
SRC_SIMULATOR	:= ../../simulator/src
SRC_ILQR		:= src
SRC_UTILS		:= ../../utils/src
INCLUDE := /usr/local/include
LIB		:= lib
LIBRARIES	:= -lyaml-cpp -lpthread

EXECUTABLE1	:= compute_ilqr
SRCS1 := $(SRC_ILQR)/main_ilqr.cpp $(SRC_PLANT)/pendulum_plant.cpp $(SRC_SIMULATOR)/simulator.cpp $(SRC_ILQR)/ilqr.cpp
SRCS3 := $(SRC_PLANT)/pendulum_plant.cpp $(SRC_SIMULATOR)/simulator.cpp $(SRC_ILQR)/ilqr.cpp

all: $(BIN)/$(EXECUTABLE1) $(OBJ)/libilqr.a

$(BIN)/$(EXECUTABLE1): $(SRCS1)
	@echo " Building ilqr ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

$(OBJ)/libilqr.a: $(OBJ)/libilqr.o
	ar rcs $@ $^

$(OBJ)/libilqr.o: $(SRCS3)
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) -c $< -o $@ $(LIBRARIES)

clean:
	@echo " Clearing..."
	-rm $(BIN)/*
	-rm $(OBJ)/*
