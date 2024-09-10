#################################################
#description: make C/C++
#author: zyh
#date: 2023-11-17
#################################################

TARGET=test

CXX=g++
CC=gcc

MAVSDK_DIR:= ../../install

SRC_DIR:= .
SRC_DIR+= include src 
#SRC_DIR+= $(MAVSDK_DIR)/include $(MAVSDK_DIR)/include/mavsdk $(MAVSDK_DIR)/include/mavsdk/plugins $(MAVSDK_DIR)/include/mavsdk/mavlink
SRC_DIR+= $(MAVSDK_DIR)/include $(MAVSDK_DIR)/include/mavsdk $(MAVSDK_DIR)/include/mavsdk/plugins

INC_DIR:= $(SRC_DIR)

SRCS:= $(foreach dir,$(SRC_DIR),$(wildcard $(dir)/*.c))
SRCS+= $(foreach dir,$(SRC_DIR),$(wildcard $(dir)/*.cpp))
INCS:= $(addprefix -I ,$(INC_DIR))

OBJS:= $(SRCS:.c=.o)
OBJS:= $(OBJS:.cpp=.o)

CFLAGS:= -Wall -Wno-address-of-packed-member -Wno-cast-align
#MAVSDK -std=c++17
CFLAGS+= -std=c++17

LIBS:= -lpthread -lm
LIBS+= -L $(MAVSDK_DIR)/lib -lmavsdk

all: $(TARGET)
%.o: %.c
	$(CXX) -c $< -o $@ $(INCS) $(CFLAGS)

%.o: %.cpp
	$(CXX) -c $< -o $@ $(INCS) $(CFLAGS)
	
$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LIBS)

clean:
	rm -f $(OBJS) $(TARGET)


