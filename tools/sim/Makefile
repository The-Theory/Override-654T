# Desktop sim: the robot's autons and Tide against fake PROS and LemLib, on the host compiler.
# Separate from the brain Makefile; see docs/sim.md

ROOT     := ../..
OUT      := out
INCLUDES := -I fake -I . -I $(ROOT)/include -I $(ROOT)/src
CXXFLAGS := -std=c++20 -O2 -g -Wall $(INCLUDES)
SRC      := $(shell find $(ROOT)/src -name '*.cpp')
HEADERS  := $(shell find fake . $(ROOT)/include/tide $(ROOT)/src -name '*.hpp' -o -name '*.h')
AUTON    ?=
WEB      ?= web/public/sim.js

# One self-contained ES module (wasm inlined) exporting createSim(); a fresh instance per run,
# since the robot's globals only start clean once. Fibers need Asyncify.
EMFLAGS  := -std=c++20 -O2 $(INCLUDES) -sASYNCIFY -sASYNCIFY_STACK_SIZE=131072 -sSTACK_SIZE=1048576 \
            -sMODULARIZE -sEXPORT_ES6 -sEXPORT_NAME=createSim -sSINGLE_FILE -sENVIRONMENT=web \
            -sINVOKE_RUN=0 -sEXIT_RUNTIME=0 -sALLOW_MEMORY_GROWTH -sEXPORTED_RUNTIME_METHODS=callMain,FS

.PHONY: run check web view

run: $(OUT)/sim
	$(OUT)/sim "$(AUTON)" --out $(OUT) --field $(ROOT)/docs/field-map.svg

check: $(OUT)/check
	$(OUT)/check

web: $(WEB)

# Builds the browser sim and serves the viewer
view: web
	cd web && bun install && bun run dev

$(WEB): main.cpp $(SRC) $(HEADERS)
	@mkdir -p $(dir $(WEB))
	em++ $(EMFLAGS) main.cpp $(SRC) -o $@

$(OUT)/sim: main.cpp $(SRC) $(HEADERS)
	@mkdir -p $(OUT)
	$(CXX) $(CXXFLAGS) main.cpp $(SRC) -o $@

$(OUT)/check: check.cpp $(HEADERS)
	@mkdir -p $(OUT)
	$(CXX) $(CXXFLAGS) check.cpp -o $@
