CXX = g++
CXXFLAGS = -std=c++17 -Wall

SFML_INCLUDE = /usr/include
SFML_LIB = /usr/lib/x86_64-linux-gnu

SFML_LIBS = -lsfml-graphics -lsfml-window -lsfml-system

TARGET = app
SRC = main.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) -I$(SFML_INCLUDE) -L$(SFML_LIB) -o $(TARGET) $(SRC) $(SFML_LIBS)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -f $(TARGET)

