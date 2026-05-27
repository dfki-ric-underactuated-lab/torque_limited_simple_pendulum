CXX		  := g++
CXX_FLAGS := -Wall -Wextra -std=c++17 -ggdb

BIN		:= bin
SRC		:= src
INCLUDE := include
LIB		:= lib
LIBRARIES	:= -lyaml-cpp -lpthread

EXECUTABLE3	:= compute_FW
EXECUTABLE4	:= compute_Dijktra_paths
#EXECUTABLE5	:= compute_Dijktra_path
EXECUTABLE6	:= compute_Dijktra_paths_parallel
SRCS3 := $(SRC)/compute_FW.cpp $(SRC)/utils.cpp 
SRCS4 := $(SRC)/compute_Dijkstra_paths.cpp $(SRC)/utils.cpp 
#SRCS5 := $(SRC)/compute_Dijkstra_path.cpp $(SRC)/utils.cpp $(SRC)/graph_class.cpp 
SRCS6 := $(SRC)/compute_Dijkstra_paths_parallel.cpp $(SRC)/utils.cpp

all: $(BIN)/$(EXECUTABLE3) $(BIN)/$(EXECUTABLE4) $(BIN)/$(EXECUTABLE6)

#run: clean all
#	clear
#	@echo " Executing..."
#	./$(BIN)/$(EXECUTABLE1)

$(BIN)/$(EXECUTABLE3): $(SRCS3)
	@echo " Building compute_FW ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

$(BIN)/$(EXECUTABLE4): $(SRCS4)
	@echo " Building compute_Dijkstra_paths ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

$(BIN)/$(EXECUTABLE6): $(SRCS6)
	@echo " Building compute_Dijkstra_paths_parallel ..."
	$(CXX) $(CXX_FLAGS) -I$(INCLUDE) -L$(LIB) $^ -o $@ $(LIBRARIES)

clean:
	@echo " Clearing..."
	-rm $(BIN)/*

