DIR_SRC = ./src/

DIR_INC = ./include/

DIR_LIB = ./

LIB_DY = -lbw_ce30v1.10

OBJS = $(DIR_SRC)test.cpp $(DIR_INC)ce30_sdk.h

test : $(OBJS)
	g++ $(DIR_SRC)test.cpp -L$(DIR_LIB) $(LIB_DY) -o test
