CXX     =   /usr/bin/g++
INCLUDE_DIRS = /usr/include/libxml2
CFLAGS  = -g -D DEBUG $(INCLUDE_DIRS:%=-I%) -std=c++14
CLIENT_LIBS = SDL2 GL GLEW png xml2 cairo unzip boost_filesystem boost_thread boost_system

all: bin/client

clean:
	rm -f bin/client obj/*.o

obj/%.o: src/%.cpp
	mkdir -p obj
	$(CXX) $(CFLAGS) -c $< -o $@

bin/client: obj/client-main.o obj/client.o obj/settings.o obj/server.o obj/xml.o \
			obj/event.o obj/play.o obj/texture.o obj/font.o obj/exception.o obj/shader.o \
			obj/str.o obj/resource.o obj/scene.o obj/lang.o obj/icosphere.o obj/io.o \
			obj/collision.o
	mkdir -p bin
	$(CXX) -o $@ $^ $(CLIENT_LIBS:%=-l%)
