all: serial main
	g++ lib/main.o lib/SerialClass.o -o bin/main.exe
	
serial: src/SerialClass.cpp
	g++ -c src/SerialClass.cpp -o lib/SerialClass.o -I./include
	
main: src/main.cpp
	g++ -c src/main.cpp -o lib/main.o -I./include
	
clean:
	rm bin/*.exe
	rm lib/*.o