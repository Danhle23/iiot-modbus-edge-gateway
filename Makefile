CXX      ?= g++
CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Werror -pedantic -Iinclude

.PHONY: all test demo clean

all: test

test:
	$(CXX) $(CXXFLAGS) tests/test_gateway.cpp -o build_test_gateway
	./build_test_gateway

demo:
	python3 tools/virtual_plc_demo.py

clean:
	rm -f build_test_gateway
