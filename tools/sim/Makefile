# Desktop sim: the robot's autons and Tide against fake PROS and LemLib, on the host compiler.
# Separate from the brain Makefile; see docs/sim.md

ROOT     := ../..
OUT      := out
CXXFLAGS := -std=c++20 -O2 -g -Wall -I fake -I . -I $(ROOT)/include -I $(ROOT)/src
SRC      := $(shell find $(ROOT)/src -name '*.cpp')
HEADERS  := $(shell find fake . $(ROOT)/include/tide $(ROOT)/src -name '*.hpp' -o -name '*.h')
AUTON    ?=

.PHONY: run check

run: $(OUT)/sim
	$(OUT)/sim "$(AUTON)" --out $(OUT) --field $(ROOT)/docs/field-map.svg

check: $(OUT)/check
	$(OUT)/check

$(OUT)/sim: main.cpp $(SRC) $(HEADERS)
	@mkdir -p $(OUT)
	$(CXX) $(CXXFLAGS) main.cpp $(SRC) -o $@

$(OUT)/check: check.cpp $(HEADERS)
	@mkdir -p $(OUT)
	$(CXX) $(CXXFLAGS) check.cpp -o $@
