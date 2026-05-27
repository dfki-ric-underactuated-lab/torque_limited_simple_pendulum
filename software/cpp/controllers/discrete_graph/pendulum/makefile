CXX		  := g++
CXX_FLAGS := -Wall -Wextra -std=c++17 -ggdb

BIN		:= bin
SRC		:= src
GRAPHSRC		:= ../graph/src
INCLUDE := include
LIB		:= lib
LIBRARIES	:= -lyaml-cpp -lpthread

EXECUTABLE1	:= explore
EXECUTABLE1b:= explore_parallel
EXECUTABLE1c:= explore_predefstates
EXECUTABLE2	:= search_graph
EXECUTABLE2b:= dijkstra_inv_tree
EXECUTABLE3	:= example3
SRCS1 := $(SRC)/pendulum_graph_construction.cpp $(GRAPHSRC)/utils.cpp $(SRC)/pendulum_utils.cpp $(SRC)/pendulum_plant.cpp 
SRCS1b := $(SRC)/pendulum_graph_construction_parallel.cpp $(GRAPHSRC)/utils.cpp $(SRC)/pendulum_utils.cpp $(SRC)/pendulum_plant.cpp 
SRCS1c := $(SRC)/pendulum_graph_construction_predefstates.cpp $(GRAPHSRC)/utils.cpp $(SRC)/pendulum_utils.cpp $(SRC)/pendulum_plant.cpp 
SRCS2 := $(SRC)/search_graph.cpp $(GRAPHSRC)/utils.cpp  $(SRC)/pendulum_utils.cpp
SRCS2b := $(SRC)/Dijkstra_inv.cpp $(GRAPHSRC)/utils.cpp  $(SRC)/pendulum_utils.cpp
SRCS3 := $(SRC)/example3.cpp

all: $(BIN)/$(EXECUTABLE1) $(BIN)/$(EXECUTABLE2) $(BIN)/$(EXECUTABLE3) $(BIN)/$(EXECUTABLE1b) $(BIN)/$(EXECUTABLE1c) $(BIN)/$(EXECUTABLE2b) 

#run: clean all
#	clear
#	@echo " Executing..."
#	./$(BIN)/$(EXECUTABLE1)

$(BIN)/$(EXECUTABLE1): $(SRCS1)
	@echo " Building explore ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

$(BIN)/$(EXECUTABLE2): $(SRCS2)
	@echo " Building search_graph ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

$(BIN)/$(EXECUTABLE3): $(SRCS3)
	@echo " Building example3 ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

$(BIN)/$(EXECUTABLE1b): $(SRCS1b)
	@echo " Building explore_parallel..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

$(BIN)/$(EXECUTABLE1c): $(SRCS1c)
	@echo " Building explore_predefstates..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

$(BIN)/$(EXECUTABLE2b): $(SRCS2b)
	@echo " Building dijkstra_inv_tree ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

clean:
	@echo " Clearing..."
	-rm $(BIN)/*

