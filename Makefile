all:
	g++ main.cpp -o agent -I./mavlink

clean: 
	rm -rf ./agent
