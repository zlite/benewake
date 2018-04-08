DIR_INC = -I, -I./include/

DIR_SRC = ./src/

MODULE = bw_ce30v1.10

all: $(MODULE).so

$(MODULE).so: $(DIR_SRC)ce30_sdk.cpp
	g++ -shared -fPIC $(DIR_SRC)ce30_sdk.cpp -o lib$(MODULE).so -lm
